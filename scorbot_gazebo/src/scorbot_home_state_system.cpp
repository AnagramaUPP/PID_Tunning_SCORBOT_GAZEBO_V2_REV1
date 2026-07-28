#include <algorithm>
#include <atomic>
#include <cstddef>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/msgs/int32.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Types.hh>

#include <sdf/Element.hh>

namespace scorbot_gazebo
{

class ScorbotHomeStateSystem final
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
public:
  void Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager & /* _eventManager */) override
  {
    this->modelEntity_ = _entity;

    const gz::sim::Model model(this->modelEntity_);

    if (!model.Valid(_ecm))
    {
      gzerr
        << "[ScorbotHomeStateSystem] El plugin no está asociado "
        << "a un modelo válido."
        << std::endl;

      this->enabled_ = false;
      return;
    }

    if (_sdf != nullptr && _sdf->HasElement("joint_names"))
    {
      const auto parsedNames = ParseStrings(
        _sdf->Get<std::string>("joint_names"));

      if (!parsedNames.empty())
      {
        this->jointNames_ = parsedNames;
      }
    }

    if (_sdf != nullptr && _sdf->HasElement("home_positions"))
    {
      const auto parsedPositions = ParseDoubles(
        _sdf->Get<std::string>("home_positions"));

      if (!parsedPositions.empty())
      {
        this->homePositions_ = parsedPositions;
      }
    }

    if (_sdf != nullptr && _sdf->HasElement("reset_cycles"))
    {
      this->resetCycles_ = std::max(
        1,
        _sdf->Get<int>("reset_cycles"));
    }

    if (_sdf != nullptr && _sdf->HasElement("log_resets"))
    {
      this->logResets_ = _sdf->Get<bool>("log_resets");
    }

    if (this->jointNames_.size() != this->homePositions_.size())
    {
      gzerr
        << "[ScorbotHomeStateSystem] La cantidad de nombres "
        << "de articulaciones no coincide con la cantidad de "
        << "posiciones home."
        << std::endl;

      this->enabled_ = false;
      return;
    }

    if (this->jointNames_.empty())
    {
      gzerr
        << "[ScorbotHomeStateSystem] No se configuraron "
        << "articulaciones."
        << std::endl;

      this->enabled_ = false;
      return;
    }

    this->resetPublisher_ =
      this->transportNode_.Advertise<gz::msgs::Int32>(
        "/scorbot/home_reset");

    if (!this->resetPublisher_)
    {
      gzerr
        << "[ScorbotHomeStateSystem] No se pudo anunciar "
        << "el tópico /scorbot/home_reset."
        << std::endl;

      this->enabled_ = false;
      return;
    }

    const bool subscribed =
      this->transportNode_.Subscribe(
        "/scorbot/pid_reset_done",
        &ScorbotHomeStateSystem::OnPidResetDone,
        this);

    if (!subscribed)
    {
      gzerr
        << "[ScorbotHomeStateSystem] No se pudo crear "
        << "la suscripción /scorbot/pid_reset_done."
        << std::endl;

      this->enabled_ = false;
      return;
    }

    if (this->ResolveJointEntities(_ecm))
    {
      // Habilita la lectura numérica para verificar el estado
      // realmente aplicado por el motor de física.
      for (const auto jointEntity : this->jointEntities_)
      {
        gz::sim::Joint joint(jointEntity);
        joint.EnablePositionCheck(_ecm);
        joint.EnableVelocityCheck(_ecm);
      }
    }

    gzmsg
      << "[ScorbotHomeStateSystem] Plugin configurado para "
      << this->jointNames_.size()
      << " articulaciones. Ciclos por reposicionamiento: "
      << this->resetCycles_
      << "."
      << std::endl;
  }

  void PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm) override
  {
    if (!this->enabled_)
    {
      return;
    }

    // Conserva el estado inicial de pausa sin provocar
    // un reposicionamiento durante la carga del modelo.
    if (!this->pauseStateKnown_)
    {
      this->pauseStateKnown_ = true;
      this->wasPaused_ = _info.paused;
      return;
    }

    // Cada transición de mundo pausado a mundo ejecutándose
    // corresponde al inicio de una nueva evaluación.
    if (this->wasPaused_ && !_info.paused)
    {
      this->remainingResetCycles_ = this->resetCycles_;
      ++this->resetCount_;

      this->activeResetId_ =
        static_cast<int>(this->resetCount_);

      this->holdCycles_ = 0;

      // La primera transición corresponde a la activación
      // inicial. Las posteriores esperan la confirmación
      // de que el controlador PID terminó de reciclarse.
      if (this->resetCount_ > 1)
      {
        this->waitingForPidReset_ = true;

        this->acknowledgedResetId_.store(
          0,
          std::memory_order_relaxed);

        gz::msgs::Int32 message;

        message.set_data(this->activeResetId_);

        if (!this->resetPublisher_.Publish(message))
        {
          gzerr
            << "[ScorbotHomeStateSystem] No se pudo publicar "
            << "el evento de reposicionamiento #"
            << this->resetCount_
            << "."
            << std::endl;
        }
      }
      else
      {
        this->waitingForPidReset_ = false;

        if (this->logResets_)
        {
          gzmsg
            << "[ScorbotHomeStateSystem] Primera transición: "
            << "activación inicial, sin notificación PID."
            << std::endl;
        }
      }

      if (this->logResets_)
      {
        gzmsg
          << "[ScorbotHomeStateSystem] Reposicionamiento #"
          << this->resetCount_
          << " solicitado."
          << std::endl;
      }
    }

    this->wasPaused_ = _info.paused;

    // No se modifica el estado mientras la física está pausada.
    if (_info.paused)
    {
      return;
    }

    const int acknowledgedId =
      this->acknowledgedResetId_.load(
        std::memory_order_relaxed);

    if (
      this->waitingForPidReset_
      && acknowledgedId == this->activeResetId_)
    {
      this->waitingForPidReset_ = false;

      // Se fuerza un último ciclo exacto antes de liberar.
      this->remainingResetCycles_ = std::max(
        this->remainingResetCycles_,
        1);

      if (this->logResets_)
      {
        gzmsg
          << "[ScorbotHomeStateSystem] Confirmación PID #"
          << acknowledgedId
          << " recibida después de "
          << this->holdCycles_
          << " ciclos. Se aplicará el reset final."
          << std::endl;
      }
    }

    if (
      this->remainingResetCycles_ <= 0
      && !this->waitingForPidReset_)
    {
      return;
    }

    if (!this->ResolveJointEntities(_ecm))
    {
      return;
    }

    // Cuando quedan resetCycles - 1 ciclos, ya transcurrió
    // un paso de física desde el primer restablecimiento.
    // Se lee aquí el estado realmente aplicado.
    if (
      this->logResets_
      && this->resetCycles_ > 1
      && this->remainingResetCycles_
         == this->resetCycles_ - 1)
    {
      std::ostringstream state;

      state
        << std::fixed
        << std::setprecision(6)
        << "[ScorbotHomeStateSystem] "
        << "Estado después del primer ciclo:";

      for (std::size_t index = 0;
           index < this->jointEntities_.size();
           ++index)
      {
        gz::sim::Joint joint(
          this->jointEntities_[index]);

        const auto position = joint.Position(_ecm);
        const auto velocity = joint.Velocity(_ecm);

        state
          << " "
          << this->jointNames_[index]
          << "=[";

        if (
          position.has_value()
          && !position->empty())
        {
          state << position->at(0);
        }
        else
        {
          state << "NA";
        }

        state << ", ";

        if (
          velocity.has_value()
          && !velocity->empty())
        {
          state << velocity->at(0);
        }
        else
        {
          state << "NA";
        }

        state << "]";
      }

      gzmsg << state.str() << std::endl;
    }

    for (std::size_t index = 0;
         index < this->jointEntities_.size();
         ++index)
    {
      gz::sim::Joint joint(this->jointEntities_[index]);

      joint.ResetPosition(
        _ecm,
        {this->homePositions_[index]});

      joint.ResetVelocity(
        _ecm,
        {0.0});
    }

    if (this->remainingResetCycles_ > 0)
    {
      --this->remainingResetCycles_;
    }

    if (this->waitingForPidReset_)
    {
      ++this->holdCycles_;
    }
  }

private:
  void OnPidResetDone(
    const gz::msgs::Int32 &_message)
  {
    this->acknowledgedResetId_.store(
      _message.data(),
      std::memory_order_relaxed);
  }

  static std::vector<std::string> ParseStrings(
    const std::string &_text)
  {
    std::vector<std::string> values;
    std::istringstream stream(_text);
    std::string value;

    while (stream >> value)
    {
      values.push_back(value);
    }

    return values;
  }

  static std::vector<double> ParseDoubles(
    const std::string &_text)
  {
    std::vector<double> values;
    std::istringstream stream(_text);
    double value = 0.0;

    while (stream >> value)
    {
      values.push_back(value);
    }

    return values;
  }

  bool ResolveJointEntities(
    gz::sim::EntityComponentManager &_ecm)
  {
    if (
      this->jointEntities_.size() == this->jointNames_.size()
      && !this->jointEntities_.empty())
    {
      return true;
    }

    this->jointEntities_.clear();

    const gz::sim::Model model(this->modelEntity_);

    for (const auto &jointName : this->jointNames_)
    {
      const gz::sim::Entity jointEntity =
        model.JointByName(_ecm, jointName);

      if (jointEntity == gz::sim::kNullEntity)
      {
        this->jointEntities_.clear();

        if (!this->jointWarningPrinted_)
        {
          gzerr
            << "[ScorbotHomeStateSystem] No se encontró todavía "
            << "la articulación: "
            << jointName
            << ". Se volverá a intentar."
            << std::endl;

          this->jointWarningPrinted_ = true;
        }

        return false;
      }

      this->jointEntities_.push_back(jointEntity);
    }

    this->jointWarningPrinted_ = false;
    return true;
  }

private:
  gz::transport::Node transportNode_;
  gz::transport::Node::Publisher resetPublisher_;

  gz::sim::Entity modelEntity_{gz::sim::kNullEntity};

  std::vector<std::string> jointNames_{
    "j1",
    "j2",
    "j3",
    "j4",
    "j5"
  };

  std::vector<double> homePositions_{
    0.000000,
    0.226002,
    -0.855270,
    0.629268,
    1.570796
  };

  std::vector<gz::sim::Entity> jointEntities_;

  int resetCycles_{2};
  int remainingResetCycles_{0};
  int activeResetId_{0};

  std::atomic<int> acknowledgedResetId_{0};

  bool waitingForPidReset_{false};

  std::size_t holdCycles_{0};

  bool enabled_{true};
  bool logResets_{true};
  bool pauseStateKnown_{false};
  bool wasPaused_{true};
  bool jointWarningPrinted_{false};

  std::size_t resetCount_{0};
};

}  // namespace scorbot_gazebo

GZ_ADD_PLUGIN(
  scorbot_gazebo::ScorbotHomeStateSystem,
  gz::sim::System,
  scorbot_gazebo::ScorbotHomeStateSystem::ISystemConfigure,
  scorbot_gazebo::ScorbotHomeStateSystem::ISystemPreUpdate)
