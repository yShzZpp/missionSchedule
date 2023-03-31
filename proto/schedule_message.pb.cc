// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: schedule_message.proto

#include "local_schedule/schedule_message.pb.h"

#include <algorithm>

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/stubs/port.h>
#include <google/protobuf/stubs/once.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/wire_format_lite_inl.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// This is a temporary google only hack
#ifdef GOOGLE_PROTOBUF_ENFORCE_UNIQUENESS
#include "third_party/protobuf/version.h"
#endif
// @@protoc_insertion_point(includes)
namespace cti {
namespace common {
class AgentScheduleMessageDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<AgentScheduleMessage>
      _instance;
} _AgentScheduleMessage_default_instance_;
}  // namespace common
}  // namespace cti
namespace protobuf_schedule_5fmessage_2eproto {
void InitDefaultsAgentScheduleMessageImpl() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

#ifdef GOOGLE_PROTOBUF_ENFORCE_UNIQUENESS
  ::google::protobuf::internal::InitProtobufDefaultsForceUnique();
#else
  ::google::protobuf::internal::InitProtobufDefaults();
#endif  // GOOGLE_PROTOBUF_ENFORCE_UNIQUENESS
  {
    void* ptr = &::cti::common::_AgentScheduleMessage_default_instance_;
    new (ptr) ::cti::common::AgentScheduleMessage();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::cti::common::AgentScheduleMessage::InitAsDefaultInstance();
}

void InitDefaultsAgentScheduleMessage() {
  static GOOGLE_PROTOBUF_DECLARE_ONCE(once);
  ::google::protobuf::GoogleOnceInit(&once, &InitDefaultsAgentScheduleMessageImpl);
}

::google::protobuf::Metadata file_level_metadata[1];
const ::google::protobuf::EnumDescriptor* file_level_enum_descriptors[2];

const ::google::protobuf::uint32 TableStruct::offsets[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::cti::common::AgentScheduleMessage, _has_bits_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::cti::common::AgentScheduleMessage, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::cti::common::AgentScheduleMessage, agent_id_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::cti::common::AgentScheduleMessage, agent_position_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::cti::common::AgentScheduleMessage, agent_decision_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::cti::common::AgentScheduleMessage, current_floor_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::cti::common::AgentScheduleMessage, agent_navigation_state_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::cti::common::AgentScheduleMessage, agent_path_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::cti::common::AgentScheduleMessage, stop_moving_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::cti::common::AgentScheduleMessage, elevator_id_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::cti::common::AgentScheduleMessage, target_floor_),
  2,
  ~0u,
  4,
  ~0u,
  3,
  ~0u,
  5,
  0,
  1,
};
static const ::google::protobuf::internal::MigrationSchema schemas[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 14, sizeof(::cti::common::AgentScheduleMessage)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&::cti::common::_AgentScheduleMessage_default_instance_),
};

void protobuf_AssignDescriptors() {
  AddDescriptors();
  ::google::protobuf::MessageFactory* factory = NULL;
  AssignDescriptors(
      "schedule_message.proto", schemas, file_default_instances, TableStruct::offsets, factory,
      file_level_metadata, file_level_enum_descriptors, NULL);
}

void protobuf_AssignDescriptorsOnce() {
  static GOOGLE_PROTOBUF_DECLARE_ONCE(once);
  ::google::protobuf::GoogleOnceInit(&once, &protobuf_AssignDescriptors);
}

void protobuf_RegisterTypes(const ::std::string&) GOOGLE_PROTOBUF_ATTRIBUTE_COLD;
void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::internal::RegisterAllTypes(file_level_metadata, 1);
}

void AddDescriptorsImpl() {
  InitDefaults();
  static const char descriptor[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
      "\n\026schedule_message.proto\022\ncti.common\"\265\004\n"
      "\024AgentScheduleMessage\022\020\n\010agent_id\030\001 \001(\005\022"
      "\032\n\016agent_position\030\002 \003(\021B\002\020\001\022J\n\016agent_dec"
      "ision\030\003 \001(\0162,.cti.common.AgentScheduleMe"
      "ssage.AgentStatus:\004MOVE\022\031\n\rcurrent_floor"
      "\030\004 \003(\021B\002\020\001\022Y\n\026agent_navigation_state\030\005 \001"
      "(\01620.cti.common.AgentScheduleMessage.Nav"
      "igationState:\007STOPPED\022\026\n\nagent_path\030\006 \003("
      "\021B\002\020\001\022\031\n\013stop_moving\030\007 \001(\010:\004true\022\023\n\013elev"
      "ator_id\030\010 \001(\t\022\024\n\014target_floor\030\t \001(\t\"!\n\013A"
      "gentStatus\022\010\n\004STOP\020\000\022\010\n\004MOVE\020\001\"\253\001\n\017Navig"
      "ationState\022\013\n\007STOPPED\020\000\022\n\n\006MOVING\020\001\022\013\n\007L"
      "IFTING\020\002\022\013\n\007DOCKING\020\003\022\016\n\nUNSTOPABLE\020\004\022\013\n"
      "\007WAITING\020\005\022\t\n\005PAUSE\020\006\022\017\n\013OCCUPY_LIFT\020\007\022\t"
      "\n\005ERROR\020\010\022\023\n\017DOCKING_BLOCKED\020\t\022\014\n\010CHARGI"
      "NG\020\n"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 604);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "schedule_message.proto", &protobuf_RegisterTypes);
}

void AddDescriptors() {
  static GOOGLE_PROTOBUF_DECLARE_ONCE(once);
  ::google::protobuf::GoogleOnceInit(&once, &AddDescriptorsImpl);
}
// Force AddDescriptors() to be called at dynamic initialization time.
struct StaticDescriptorInitializer {
  StaticDescriptorInitializer() {
    AddDescriptors();
  }
} static_descriptor_initializer;
}  // namespace protobuf_schedule_5fmessage_2eproto
namespace cti {
namespace common {
const ::google::protobuf::EnumDescriptor* AgentScheduleMessage_AgentStatus_descriptor() {
  protobuf_schedule_5fmessage_2eproto::protobuf_AssignDescriptorsOnce();
  return protobuf_schedule_5fmessage_2eproto::file_level_enum_descriptors[0];
}
bool AgentScheduleMessage_AgentStatus_IsValid(int value) {
  switch (value) {
    case 0:
    case 1:
      return true;
    default:
      return false;
  }
}

#if !defined(_MSC_VER) || _MSC_VER >= 1900
const AgentScheduleMessage_AgentStatus AgentScheduleMessage::STOP;
const AgentScheduleMessage_AgentStatus AgentScheduleMessage::MOVE;
const AgentScheduleMessage_AgentStatus AgentScheduleMessage::AgentStatus_MIN;
const AgentScheduleMessage_AgentStatus AgentScheduleMessage::AgentStatus_MAX;
const int AgentScheduleMessage::AgentStatus_ARRAYSIZE;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900
const ::google::protobuf::EnumDescriptor* AgentScheduleMessage_NavigationState_descriptor() {
  protobuf_schedule_5fmessage_2eproto::protobuf_AssignDescriptorsOnce();
  return protobuf_schedule_5fmessage_2eproto::file_level_enum_descriptors[1];
}
bool AgentScheduleMessage_NavigationState_IsValid(int value) {
  switch (value) {
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
    case 8:
    case 9:
    case 10:
      return true;
    default:
      return false;
  }
}

#if !defined(_MSC_VER) || _MSC_VER >= 1900
const AgentScheduleMessage_NavigationState AgentScheduleMessage::STOPPED;
const AgentScheduleMessage_NavigationState AgentScheduleMessage::MOVING;
const AgentScheduleMessage_NavigationState AgentScheduleMessage::LIFTING;
const AgentScheduleMessage_NavigationState AgentScheduleMessage::DOCKING;
const AgentScheduleMessage_NavigationState AgentScheduleMessage::UNSTOPABLE;
const AgentScheduleMessage_NavigationState AgentScheduleMessage::WAITING;
const AgentScheduleMessage_NavigationState AgentScheduleMessage::PAUSE;
const AgentScheduleMessage_NavigationState AgentScheduleMessage::OCCUPY_LIFT;
const AgentScheduleMessage_NavigationState AgentScheduleMessage::ERROR;
const AgentScheduleMessage_NavigationState AgentScheduleMessage::DOCKING_BLOCKED;
const AgentScheduleMessage_NavigationState AgentScheduleMessage::CHARGING;
const AgentScheduleMessage_NavigationState AgentScheduleMessage::NavigationState_MIN;
const AgentScheduleMessage_NavigationState AgentScheduleMessage::NavigationState_MAX;
const int AgentScheduleMessage::NavigationState_ARRAYSIZE;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

// ===================================================================

void AgentScheduleMessage::InitAsDefaultInstance() {
}
#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int AgentScheduleMessage::kAgentIdFieldNumber;
const int AgentScheduleMessage::kAgentPositionFieldNumber;
const int AgentScheduleMessage::kAgentDecisionFieldNumber;
const int AgentScheduleMessage::kCurrentFloorFieldNumber;
const int AgentScheduleMessage::kAgentNavigationStateFieldNumber;
const int AgentScheduleMessage::kAgentPathFieldNumber;
const int AgentScheduleMessage::kStopMovingFieldNumber;
const int AgentScheduleMessage::kElevatorIdFieldNumber;
const int AgentScheduleMessage::kTargetFloorFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

AgentScheduleMessage::AgentScheduleMessage()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  if (GOOGLE_PREDICT_TRUE(this != internal_default_instance())) {
    ::protobuf_schedule_5fmessage_2eproto::InitDefaultsAgentScheduleMessage();
  }
  SharedCtor();
  // @@protoc_insertion_point(constructor:cti.common.AgentScheduleMessage)
}
AgentScheduleMessage::AgentScheduleMessage(const AgentScheduleMessage& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      _has_bits_(from._has_bits_),
      _cached_size_(0),
      agent_position_(from.agent_position_),
      current_floor_(from.current_floor_),
      agent_path_(from.agent_path_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  elevator_id_.UnsafeSetDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  if (from.has_elevator_id()) {
    elevator_id_.AssignWithDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), from.elevator_id_);
  }
  target_floor_.UnsafeSetDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  if (from.has_target_floor()) {
    target_floor_.AssignWithDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), from.target_floor_);
  }
  ::memcpy(&agent_id_, &from.agent_id_,
    static_cast<size_t>(reinterpret_cast<char*>(&stop_moving_) -
    reinterpret_cast<char*>(&agent_id_)) + sizeof(stop_moving_));
  // @@protoc_insertion_point(copy_constructor:cti.common.AgentScheduleMessage)
}

void AgentScheduleMessage::SharedCtor() {
  _cached_size_ = 0;
  elevator_id_.UnsafeSetDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  target_floor_.UnsafeSetDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  ::memset(&agent_id_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&agent_navigation_state_) -
      reinterpret_cast<char*>(&agent_id_)) + sizeof(agent_navigation_state_));
  agent_decision_ = 1;
  stop_moving_ = true;
}

AgentScheduleMessage::~AgentScheduleMessage() {
  // @@protoc_insertion_point(destructor:cti.common.AgentScheduleMessage)
  SharedDtor();
}

void AgentScheduleMessage::SharedDtor() {
  elevator_id_.DestroyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  target_floor_.DestroyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}

void AgentScheduleMessage::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* AgentScheduleMessage::descriptor() {
  ::protobuf_schedule_5fmessage_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_schedule_5fmessage_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const AgentScheduleMessage& AgentScheduleMessage::default_instance() {
  ::protobuf_schedule_5fmessage_2eproto::InitDefaultsAgentScheduleMessage();
  return *internal_default_instance();
}

AgentScheduleMessage* AgentScheduleMessage::New(::google::protobuf::Arena* arena) const {
  AgentScheduleMessage* n = new AgentScheduleMessage;
  if (arena != NULL) {
    arena->Own(n);
  }
  return n;
}

void AgentScheduleMessage::Clear() {
// @@protoc_insertion_point(message_clear_start:cti.common.AgentScheduleMessage)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  agent_position_.Clear();
  current_floor_.Clear();
  agent_path_.Clear();
  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 3u) {
    if (cached_has_bits & 0x00000001u) {
      GOOGLE_DCHECK(!elevator_id_.IsDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited()));
      (*elevator_id_.UnsafeRawStringPointer())->clear();
    }
    if (cached_has_bits & 0x00000002u) {
      GOOGLE_DCHECK(!target_floor_.IsDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited()));
      (*target_floor_.UnsafeRawStringPointer())->clear();
    }
  }
  if (cached_has_bits & 60u) {
    ::memset(&agent_id_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&agent_navigation_state_) -
        reinterpret_cast<char*>(&agent_id_)) + sizeof(agent_navigation_state_));
    agent_decision_ = 1;
    stop_moving_ = true;
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

bool AgentScheduleMessage::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:cti.common.AgentScheduleMessage)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // optional int32 agent_id = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(8u /* 8 & 0xFF */)) {
          set_has_agent_id();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::int32, ::google::protobuf::internal::WireFormatLite::TYPE_INT32>(
                 input, &agent_id_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // repeated sint32 agent_position = 2 [packed = true];
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(18u /* 18 & 0xFF */)) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadPackedPrimitive<
                   ::google::protobuf::int32, ::google::protobuf::internal::WireFormatLite::TYPE_SINT32>(
                 input, this->mutable_agent_position())));
        } else if (
            static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(16u /* 16 & 0xFF */)) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadRepeatedPrimitiveNoInline<
                   ::google::protobuf::int32, ::google::protobuf::internal::WireFormatLite::TYPE_SINT32>(
                 1, 18u, input, this->mutable_agent_position())));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional .cti.common.AgentScheduleMessage.AgentStatus agent_decision = 3 [default = MOVE];
      case 3: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(24u /* 24 & 0xFF */)) {
          int value;
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   int, ::google::protobuf::internal::WireFormatLite::TYPE_ENUM>(
                 input, &value)));
          if (::cti::common::AgentScheduleMessage_AgentStatus_IsValid(value)) {
            set_agent_decision(static_cast< ::cti::common::AgentScheduleMessage_AgentStatus >(value));
          } else {
            mutable_unknown_fields()->AddVarint(
                3, static_cast< ::google::protobuf::uint64>(value));
          }
        } else {
          goto handle_unusual;
        }
        break;
      }

      // repeated sint32 current_floor = 4 [packed = true];
      case 4: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(34u /* 34 & 0xFF */)) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadPackedPrimitive<
                   ::google::protobuf::int32, ::google::protobuf::internal::WireFormatLite::TYPE_SINT32>(
                 input, this->mutable_current_floor())));
        } else if (
            static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(32u /* 32 & 0xFF */)) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadRepeatedPrimitiveNoInline<
                   ::google::protobuf::int32, ::google::protobuf::internal::WireFormatLite::TYPE_SINT32>(
                 1, 34u, input, this->mutable_current_floor())));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional .cti.common.AgentScheduleMessage.NavigationState agent_navigation_state = 5 [default = STOPPED];
      case 5: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(40u /* 40 & 0xFF */)) {
          int value;
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   int, ::google::protobuf::internal::WireFormatLite::TYPE_ENUM>(
                 input, &value)));
          if (::cti::common::AgentScheduleMessage_NavigationState_IsValid(value)) {
            set_agent_navigation_state(static_cast< ::cti::common::AgentScheduleMessage_NavigationState >(value));
          } else {
            mutable_unknown_fields()->AddVarint(
                5, static_cast< ::google::protobuf::uint64>(value));
          }
        } else {
          goto handle_unusual;
        }
        break;
      }

      // repeated sint32 agent_path = 6 [packed = true];
      case 6: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(50u /* 50 & 0xFF */)) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadPackedPrimitive<
                   ::google::protobuf::int32, ::google::protobuf::internal::WireFormatLite::TYPE_SINT32>(
                 input, this->mutable_agent_path())));
        } else if (
            static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(48u /* 48 & 0xFF */)) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadRepeatedPrimitiveNoInline<
                   ::google::protobuf::int32, ::google::protobuf::internal::WireFormatLite::TYPE_SINT32>(
                 1, 50u, input, this->mutable_agent_path())));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional bool stop_moving = 7 [default = true];
      case 7: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(56u /* 56 & 0xFF */)) {
          set_has_stop_moving();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   bool, ::google::protobuf::internal::WireFormatLite::TYPE_BOOL>(
                 input, &stop_moving_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional string elevator_id = 8;
      case 8: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(66u /* 66 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadString(
                input, this->mutable_elevator_id()));
          ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
            this->elevator_id().data(), static_cast<int>(this->elevator_id().length()),
            ::google::protobuf::internal::WireFormat::PARSE,
            "cti.common.AgentScheduleMessage.elevator_id");
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional string target_floor = 9;
      case 9: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(74u /* 74 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadString(
                input, this->mutable_target_floor()));
          ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
            this->target_floor().data(), static_cast<int>(this->target_floor().length()),
            ::google::protobuf::internal::WireFormat::PARSE,
            "cti.common.AgentScheduleMessage.target_floor");
        } else {
          goto handle_unusual;
        }
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, _internal_metadata_.mutable_unknown_fields()));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:cti.common.AgentScheduleMessage)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:cti.common.AgentScheduleMessage)
  return false;
#undef DO_
}

void AgentScheduleMessage::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:cti.common.AgentScheduleMessage)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional int32 agent_id = 1;
  if (cached_has_bits & 0x00000004u) {
    ::google::protobuf::internal::WireFormatLite::WriteInt32(1, this->agent_id(), output);
  }

  // repeated sint32 agent_position = 2 [packed = true];
  if (this->agent_position_size() > 0) {
    ::google::protobuf::internal::WireFormatLite::WriteTag(2, ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED, output);
    output->WriteVarint32(static_cast< ::google::protobuf::uint32>(
        _agent_position_cached_byte_size_));
  }
  for (int i = 0, n = this->agent_position_size(); i < n; i++) {
    ::google::protobuf::internal::WireFormatLite::WriteSInt32NoTag(
      this->agent_position(i), output);
  }

  // optional .cti.common.AgentScheduleMessage.AgentStatus agent_decision = 3 [default = MOVE];
  if (cached_has_bits & 0x00000010u) {
    ::google::protobuf::internal::WireFormatLite::WriteEnum(
      3, this->agent_decision(), output);
  }

  // repeated sint32 current_floor = 4 [packed = true];
  if (this->current_floor_size() > 0) {
    ::google::protobuf::internal::WireFormatLite::WriteTag(4, ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED, output);
    output->WriteVarint32(static_cast< ::google::protobuf::uint32>(
        _current_floor_cached_byte_size_));
  }
  for (int i = 0, n = this->current_floor_size(); i < n; i++) {
    ::google::protobuf::internal::WireFormatLite::WriteSInt32NoTag(
      this->current_floor(i), output);
  }

  // optional .cti.common.AgentScheduleMessage.NavigationState agent_navigation_state = 5 [default = STOPPED];
  if (cached_has_bits & 0x00000008u) {
    ::google::protobuf::internal::WireFormatLite::WriteEnum(
      5, this->agent_navigation_state(), output);
  }

  // repeated sint32 agent_path = 6 [packed = true];
  if (this->agent_path_size() > 0) {
    ::google::protobuf::internal::WireFormatLite::WriteTag(6, ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED, output);
    output->WriteVarint32(static_cast< ::google::protobuf::uint32>(
        _agent_path_cached_byte_size_));
  }
  for (int i = 0, n = this->agent_path_size(); i < n; i++) {
    ::google::protobuf::internal::WireFormatLite::WriteSInt32NoTag(
      this->agent_path(i), output);
  }

  // optional bool stop_moving = 7 [default = true];
  if (cached_has_bits & 0x00000020u) {
    ::google::protobuf::internal::WireFormatLite::WriteBool(7, this->stop_moving(), output);
  }

  // optional string elevator_id = 8;
  if (cached_has_bits & 0x00000001u) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->elevator_id().data(), static_cast<int>(this->elevator_id().length()),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "cti.common.AgentScheduleMessage.elevator_id");
    ::google::protobuf::internal::WireFormatLite::WriteStringMaybeAliased(
      8, this->elevator_id(), output);
  }

  // optional string target_floor = 9;
  if (cached_has_bits & 0x00000002u) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->target_floor().data(), static_cast<int>(this->target_floor().length()),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "cti.common.AgentScheduleMessage.target_floor");
    ::google::protobuf::internal::WireFormatLite::WriteStringMaybeAliased(
      9, this->target_floor(), output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        _internal_metadata_.unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:cti.common.AgentScheduleMessage)
}

::google::protobuf::uint8* AgentScheduleMessage::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:cti.common.AgentScheduleMessage)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional int32 agent_id = 1;
  if (cached_has_bits & 0x00000004u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteInt32ToArray(1, this->agent_id(), target);
  }

  // repeated sint32 agent_position = 2 [packed = true];
  if (this->agent_position_size() > 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteTagToArray(
      2,
      ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED,
      target);
    target = ::google::protobuf::io::CodedOutputStream::WriteVarint32ToArray(
        static_cast< ::google::protobuf::int32>(
            _agent_position_cached_byte_size_), target);
    target = ::google::protobuf::internal::WireFormatLite::
      WriteSInt32NoTagToArray(this->agent_position_, target);
  }

  // optional .cti.common.AgentScheduleMessage.AgentStatus agent_decision = 3 [default = MOVE];
  if (cached_has_bits & 0x00000010u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteEnumToArray(
      3, this->agent_decision(), target);
  }

  // repeated sint32 current_floor = 4 [packed = true];
  if (this->current_floor_size() > 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteTagToArray(
      4,
      ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED,
      target);
    target = ::google::protobuf::io::CodedOutputStream::WriteVarint32ToArray(
        static_cast< ::google::protobuf::int32>(
            _current_floor_cached_byte_size_), target);
    target = ::google::protobuf::internal::WireFormatLite::
      WriteSInt32NoTagToArray(this->current_floor_, target);
  }

  // optional .cti.common.AgentScheduleMessage.NavigationState agent_navigation_state = 5 [default = STOPPED];
  if (cached_has_bits & 0x00000008u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteEnumToArray(
      5, this->agent_navigation_state(), target);
  }

  // repeated sint32 agent_path = 6 [packed = true];
  if (this->agent_path_size() > 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteTagToArray(
      6,
      ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED,
      target);
    target = ::google::protobuf::io::CodedOutputStream::WriteVarint32ToArray(
        static_cast< ::google::protobuf::int32>(
            _agent_path_cached_byte_size_), target);
    target = ::google::protobuf::internal::WireFormatLite::
      WriteSInt32NoTagToArray(this->agent_path_, target);
  }

  // optional bool stop_moving = 7 [default = true];
  if (cached_has_bits & 0x00000020u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteBoolToArray(7, this->stop_moving(), target);
  }

  // optional string elevator_id = 8;
  if (cached_has_bits & 0x00000001u) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->elevator_id().data(), static_cast<int>(this->elevator_id().length()),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "cti.common.AgentScheduleMessage.elevator_id");
    target =
      ::google::protobuf::internal::WireFormatLite::WriteStringToArray(
        8, this->elevator_id(), target);
  }

  // optional string target_floor = 9;
  if (cached_has_bits & 0x00000002u) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->target_floor().data(), static_cast<int>(this->target_floor().length()),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "cti.common.AgentScheduleMessage.target_floor");
    target =
      ::google::protobuf::internal::WireFormatLite::WriteStringToArray(
        9, this->target_floor(), target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:cti.common.AgentScheduleMessage)
  return target;
}

size_t AgentScheduleMessage::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:cti.common.AgentScheduleMessage)
  size_t total_size = 0;

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        _internal_metadata_.unknown_fields());
  }
  // repeated sint32 agent_position = 2 [packed = true];
  {
    size_t data_size = ::google::protobuf::internal::WireFormatLite::
      SInt32Size(this->agent_position_);
    if (data_size > 0) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::Int32Size(
            static_cast< ::google::protobuf::int32>(data_size));
    }
    int cached_size = ::google::protobuf::internal::ToCachedSize(data_size);
    GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
    _agent_position_cached_byte_size_ = cached_size;
    GOOGLE_SAFE_CONCURRENT_WRITES_END();
    total_size += data_size;
  }

  // repeated sint32 current_floor = 4 [packed = true];
  {
    size_t data_size = ::google::protobuf::internal::WireFormatLite::
      SInt32Size(this->current_floor_);
    if (data_size > 0) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::Int32Size(
            static_cast< ::google::protobuf::int32>(data_size));
    }
    int cached_size = ::google::protobuf::internal::ToCachedSize(data_size);
    GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
    _current_floor_cached_byte_size_ = cached_size;
    GOOGLE_SAFE_CONCURRENT_WRITES_END();
    total_size += data_size;
  }

  // repeated sint32 agent_path = 6 [packed = true];
  {
    size_t data_size = ::google::protobuf::internal::WireFormatLite::
      SInt32Size(this->agent_path_);
    if (data_size > 0) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::Int32Size(
            static_cast< ::google::protobuf::int32>(data_size));
    }
    int cached_size = ::google::protobuf::internal::ToCachedSize(data_size);
    GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
    _agent_path_cached_byte_size_ = cached_size;
    GOOGLE_SAFE_CONCURRENT_WRITES_END();
    total_size += data_size;
  }

  if (_has_bits_[0 / 32] & 63u) {
    // optional string elevator_id = 8;
    if (has_elevator_id()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::StringSize(
          this->elevator_id());
    }

    // optional string target_floor = 9;
    if (has_target_floor()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::StringSize(
          this->target_floor());
    }

    // optional int32 agent_id = 1;
    if (has_agent_id()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::Int32Size(
          this->agent_id());
    }

    // optional .cti.common.AgentScheduleMessage.NavigationState agent_navigation_state = 5 [default = STOPPED];
    if (has_agent_navigation_state()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::EnumSize(this->agent_navigation_state());
    }

    // optional .cti.common.AgentScheduleMessage.AgentStatus agent_decision = 3 [default = MOVE];
    if (has_agent_decision()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::EnumSize(this->agent_decision());
    }

    // optional bool stop_moving = 7 [default = true];
    if (has_stop_moving()) {
      total_size += 1 + 1;
    }

  }
  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = cached_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void AgentScheduleMessage::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:cti.common.AgentScheduleMessage)
  GOOGLE_DCHECK_NE(&from, this);
  const AgentScheduleMessage* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const AgentScheduleMessage>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:cti.common.AgentScheduleMessage)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:cti.common.AgentScheduleMessage)
    MergeFrom(*source);
  }
}

void AgentScheduleMessage::MergeFrom(const AgentScheduleMessage& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:cti.common.AgentScheduleMessage)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  agent_position_.MergeFrom(from.agent_position_);
  current_floor_.MergeFrom(from.current_floor_);
  agent_path_.MergeFrom(from.agent_path_);
  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 63u) {
    if (cached_has_bits & 0x00000001u) {
      set_has_elevator_id();
      elevator_id_.AssignWithDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), from.elevator_id_);
    }
    if (cached_has_bits & 0x00000002u) {
      set_has_target_floor();
      target_floor_.AssignWithDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), from.target_floor_);
    }
    if (cached_has_bits & 0x00000004u) {
      agent_id_ = from.agent_id_;
    }
    if (cached_has_bits & 0x00000008u) {
      agent_navigation_state_ = from.agent_navigation_state_;
    }
    if (cached_has_bits & 0x00000010u) {
      agent_decision_ = from.agent_decision_;
    }
    if (cached_has_bits & 0x00000020u) {
      stop_moving_ = from.stop_moving_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void AgentScheduleMessage::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:cti.common.AgentScheduleMessage)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void AgentScheduleMessage::CopyFrom(const AgentScheduleMessage& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:cti.common.AgentScheduleMessage)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool AgentScheduleMessage::IsInitialized() const {
  return true;
}

void AgentScheduleMessage::Swap(AgentScheduleMessage* other) {
  if (other == this) return;
  InternalSwap(other);
}
void AgentScheduleMessage::InternalSwap(AgentScheduleMessage* other) {
  using std::swap;
  agent_position_.InternalSwap(&other->agent_position_);
  current_floor_.InternalSwap(&other->current_floor_);
  agent_path_.InternalSwap(&other->agent_path_);
  elevator_id_.Swap(&other->elevator_id_);
  target_floor_.Swap(&other->target_floor_);
  swap(agent_id_, other->agent_id_);
  swap(agent_navigation_state_, other->agent_navigation_state_);
  swap(agent_decision_, other->agent_decision_);
  swap(stop_moving_, other->stop_moving_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_cached_size_, other->_cached_size_);
}

::google::protobuf::Metadata AgentScheduleMessage::GetMetadata() const {
  protobuf_schedule_5fmessage_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_schedule_5fmessage_2eproto::file_level_metadata[kIndexInFileMessages];
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace common
}  // namespace cti

// @@protoc_insertion_point(global_scope)