#include "d1_manipulation/arm_task_state_machine.hpp"

#include <boost/sml.hpp>

#include <utility>

namespace d1_manipulation
{
namespace sml = boost::sml;
namespace
{
struct Initializing {};
struct ReadyStowed {};
struct ReadyCarry {};
struct Picking {};
struct DroppingEmpty {};
struct DroppingHeld {};
struct RecoveringToStowed {};
struct RecoveringToCarry {};
struct Faulted {};

struct InitializationSucceeded {};
struct InitializationFailed {};
struct StartPick {};
struct PickSucceeded {};
struct PickRepositionRequired {};
struct StartDrop {};
struct DropSucceeded {};
struct DropRepositionRequired {};
struct RecoverySucceeded {};
struct Fault {};

struct Context
{
  ArmTaskSnapshot snapshot;
};

struct Definition
{
  auto operator()() const
  {
    using namespace sml;
    const auto ready_stowed = [](Context& value) {
        value.snapshot = {ArmTaskState::READY_STOWED, PayloadState::EMPTY,
          CanonicalPose::STOWED};
      };
    const auto ready_carry = [](Context& value) {
        value.snapshot = {ArmTaskState::READY_CARRY, PayloadState::HELD,
          CanonicalPose::CARRY};
      };
    const auto picking = [](Context& value) {
        value.snapshot = {ArmTaskState::PICKING, PayloadState::EMPTY,
          CanonicalPose::OTHER};
      };
    const auto dropping_empty = [](Context& value) {
        value.snapshot = {ArmTaskState::DROPPING, PayloadState::EMPTY,
          CanonicalPose::OTHER};
      };
    const auto dropping_held = [](Context& value) {
        value.snapshot = {ArmTaskState::DROPPING, PayloadState::HELD,
          CanonicalPose::OTHER};
      };
    const auto recovering_stowed = [](Context& value) {
        value.snapshot = {ArmTaskState::RECOVERING_TO_STOWED, PayloadState::EMPTY,
          CanonicalPose::OTHER};
      };
    const auto recovering_carry = [](Context& value) {
        value.snapshot = {ArmTaskState::RECOVERING_TO_CARRY, PayloadState::HELD,
          CanonicalPose::OTHER};
      };
    const auto faulted = [](Context& value) {
        value.snapshot.state = ArmTaskState::FAULTED;
        value.snapshot.pose = CanonicalPose::UNKNOWN;
        value.snapshot.payload = PayloadState::UNKNOWN;
      };

    return make_transition_table(
      *state<Initializing> + event<InitializationSucceeded> / ready_stowed = state<ReadyStowed>,
       state<Initializing> + event<InitializationFailed> / faulted = state<Faulted>,
       state<ReadyStowed> + event<StartPick> / picking = state<Picking>,
       state<Picking> + event<PickSucceeded> / ready_carry = state<ReadyCarry>,
       state<Picking> + event<PickRepositionRequired> / recovering_stowed =
         state<RecoveringToStowed>,
       state<RecoveringToStowed> + event<RecoverySucceeded> / ready_stowed =
         state<ReadyStowed>,
       state<ReadyStowed> + event<StartDrop> / dropping_empty = state<DroppingEmpty>,
       state<DroppingEmpty> + event<DropSucceeded> / ready_stowed = state<ReadyStowed>,
       state<DroppingEmpty> + event<DropRepositionRequired> / recovering_stowed =
         state<RecoveringToStowed>,
       state<ReadyCarry> + event<StartDrop> / dropping_held = state<DroppingHeld>,
       state<DroppingHeld> + event<DropSucceeded> / ready_stowed = state<ReadyStowed>,
       state<DroppingHeld> + event<DropRepositionRequired> / recovering_carry =
         state<RecoveringToCarry>,
       state<RecoveringToCarry> + event<RecoverySucceeded> / ready_carry =
         state<ReadyCarry>,
       state<Initializing> + event<Fault> / faulted = state<Faulted>,
       state<ReadyStowed> + event<Fault> / faulted = state<Faulted>,
       state<ReadyCarry> + event<Fault> / faulted = state<Faulted>,
       state<Picking> + event<Fault> / faulted = state<Faulted>,
       state<DroppingEmpty> + event<Fault> / faulted = state<Faulted>,
       state<DroppingHeld> + event<Fault> / faulted = state<Faulted>,
       state<RecoveringToStowed> + event<Fault> / faulted = state<Faulted>,
       state<RecoveringToCarry> + event<Fault> / faulted = state<Faulted>,
       state<Faulted> + event<Fault> / faulted = state<Faulted>
    );
  }
};
}  // namespace

class ArmTaskStateMachine::Impl
{
public:
  Context context;
  sml::sm<Definition> machine{context};
};

ArmTaskStateMachine::ArmTaskStateMachine() : impl_(std::make_unique<Impl>()) {}
ArmTaskStateMachine::~ArmTaskStateMachine() = default;
ArmTaskStateMachine::ArmTaskStateMachine(ArmTaskStateMachine&&) noexcept = default;
ArmTaskStateMachine& ArmTaskStateMachine::operator=(ArmTaskStateMachine&&) noexcept = default;

bool ArmTaskStateMachine::process(ArmTaskEvent event)
{
  switch (event) {
    case ArmTaskEvent::INITIALIZATION_SUCCEEDED:
      return impl_->machine.process_event(InitializationSucceeded{});
    case ArmTaskEvent::INITIALIZATION_FAILED:
      return impl_->machine.process_event(InitializationFailed{});
    case ArmTaskEvent::START_PICK:
      return impl_->machine.process_event(StartPick{});
    case ArmTaskEvent::PICK_SUCCEEDED:
      return impl_->machine.process_event(PickSucceeded{});
    case ArmTaskEvent::PICK_REPOSITION_REQUIRED:
      return impl_->machine.process_event(PickRepositionRequired{});
    case ArmTaskEvent::START_DROP:
      return impl_->machine.process_event(StartDrop{});
    case ArmTaskEvent::DROP_SUCCEEDED:
      return impl_->machine.process_event(DropSucceeded{});
    case ArmTaskEvent::DROP_REPOSITION_REQUIRED:
      return impl_->machine.process_event(DropRepositionRequired{});
    case ArmTaskEvent::RECOVERY_SUCCEEDED:
      return impl_->machine.process_event(RecoverySucceeded{});
    case ArmTaskEvent::UPDATE_PHASE: {
      const auto state = impl_->context.snapshot.state;
      return state == ArmTaskState::PICKING || state == ArmTaskState::DROPPING ||
        state == ArmTaskState::RECOVERING_TO_STOWED ||
        state == ArmTaskState::RECOVERING_TO_CARRY;
    }
    case ArmTaskEvent::FAULT:
      return impl_->machine.process_event(Fault{});
  }
  return false;
}

ArmTaskSnapshot ArmTaskStateMachine::snapshot() const {return impl_->context.snapshot;}

const char* ArmTaskStateMachine::name(ArmTaskState state)
{
  switch (state) {
    case ArmTaskState::INITIALIZING: return "INITIALIZING";
    case ArmTaskState::READY_STOWED: return "READY_STOWED";
    case ArmTaskState::READY_CARRY: return "READY_CARRY";
    case ArmTaskState::PICKING: return "PICKING";
    case ArmTaskState::DROPPING: return "DROPPING";
    case ArmTaskState::RECOVERING_TO_STOWED: return "RECOVERING_TO_STOWED";
    case ArmTaskState::RECOVERING_TO_CARRY: return "RECOVERING_TO_CARRY";
    case ArmTaskState::FAULTED: return "FAULTED";
  }
  return "UNKNOWN";
}
}  // namespace d1_manipulation
