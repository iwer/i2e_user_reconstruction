// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ConnectionMessage.proto

#ifndef PROTOBUF_ConnectionMessage_2eproto__INCLUDED
#define PROTOBUF_ConnectionMessage_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 3000000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 3000000 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)

// Internal implementation detail -- do not call these.
void protobuf_AddDesc_ConnectionMessage_2eproto();
void protobuf_AssignDesc_ConnectionMessage_2eproto();
void protobuf_ShutdownFile_ConnectionMessage_2eproto();

class ConnectionMessage;

// ===================================================================

class ConnectionMessage : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:ConnectionMessage) */ {
 public:
  ConnectionMessage();
  virtual ~ConnectionMessage();

  ConnectionMessage(const ConnectionMessage& from);

  inline ConnectionMessage& operator=(const ConnectionMessage& from) {
    CopyFrom(from);
    return *this;
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const ConnectionMessage& default_instance();

  void Swap(ConnectionMessage* other);

  // implements Message ----------------------------------------------

  inline ConnectionMessage* New() const { return New(NULL); }

  ConnectionMessage* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const ConnectionMessage& from);
  void MergeFrom(const ConnectionMessage& from);
  void Clear();
  bool IsInitialized() const;

  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const {
    return InternalSerializeWithCachedSizesToArray(false, output);
  }
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  void InternalSwap(ConnectionMessage* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return _internal_metadata_.arena();
  }
  inline void* MaybeArenaPtr() const {
    return _internal_metadata_.raw_arena_ptr();
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // optional bool is_leader = 1;
  void clear_is_leader();
  static const int kIsLeaderFieldNumber = 1;
  bool is_leader() const;
  void set_is_leader(bool value);

  // optional bool use_point_cloud = 2;
  void clear_use_point_cloud();
  static const int kUsePointCloudFieldNumber = 2;
  bool use_point_cloud() const;
  void set_use_point_cloud(bool value);

  // optional uint32 video_height = 3;
  void clear_video_height();
  static const int kVideoHeightFieldNumber = 3;
  ::google::protobuf::uint32 video_height() const;
  void set_video_height(::google::protobuf::uint32 value);

  // optional uint32 video_width = 4;
  void clear_video_width();
  static const int kVideoWidthFieldNumber = 4;
  ::google::protobuf::uint32 video_width() const;
  void set_video_width(::google::protobuf::uint32 value);

  // optional uint32 depth_height = 5;
  void clear_depth_height();
  static const int kDepthHeightFieldNumber = 5;
  ::google::protobuf::uint32 depth_height() const;
  void set_depth_height(::google::protobuf::uint32 value);

  // optional uint32 depth_width = 6;
  void clear_depth_width();
  static const int kDepthWidthFieldNumber = 6;
  ::google::protobuf::uint32 depth_width() const;
  void set_depth_width(::google::protobuf::uint32 value);

  // optional uint32 message_size = 7;
  void clear_message_size();
  static const int kMessageSizeFieldNumber = 7;
  ::google::protobuf::uint32 message_size() const;
  void set_message_size(::google::protobuf::uint32 value);

  // @@protoc_insertion_point(class_scope:ConnectionMessage)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  bool _is_default_instance_;
  bool is_leader_;
  bool use_point_cloud_;
  ::google::protobuf::uint32 video_height_;
  ::google::protobuf::uint32 video_width_;
  ::google::protobuf::uint32 depth_height_;
  ::google::protobuf::uint32 depth_width_;
  ::google::protobuf::uint32 message_size_;
  mutable int _cached_size_;
  friend void  protobuf_AddDesc_ConnectionMessage_2eproto();
  friend void protobuf_AssignDesc_ConnectionMessage_2eproto();
  friend void protobuf_ShutdownFile_ConnectionMessage_2eproto();

  void InitAsDefaultInstance();
  static ConnectionMessage* default_instance_;
};
// ===================================================================


// ===================================================================

#if !PROTOBUF_INLINE_NOT_IN_HEADERS
// ConnectionMessage

// optional bool is_leader = 1;
inline void ConnectionMessage::clear_is_leader() {
  is_leader_ = false;
}
inline bool ConnectionMessage::is_leader() const {
  // @@protoc_insertion_point(field_get:ConnectionMessage.is_leader)
  return is_leader_;
}
inline void ConnectionMessage::set_is_leader(bool value) {
  
  is_leader_ = value;
  // @@protoc_insertion_point(field_set:ConnectionMessage.is_leader)
}

// optional bool use_point_cloud = 2;
inline void ConnectionMessage::clear_use_point_cloud() {
  use_point_cloud_ = false;
}
inline bool ConnectionMessage::use_point_cloud() const {
  // @@protoc_insertion_point(field_get:ConnectionMessage.use_point_cloud)
  return use_point_cloud_;
}
inline void ConnectionMessage::set_use_point_cloud(bool value) {
  
  use_point_cloud_ = value;
  // @@protoc_insertion_point(field_set:ConnectionMessage.use_point_cloud)
}

// optional uint32 video_height = 3;
inline void ConnectionMessage::clear_video_height() {
  video_height_ = 0u;
}
inline ::google::protobuf::uint32 ConnectionMessage::video_height() const {
  // @@protoc_insertion_point(field_get:ConnectionMessage.video_height)
  return video_height_;
}
inline void ConnectionMessage::set_video_height(::google::protobuf::uint32 value) {
  
  video_height_ = value;
  // @@protoc_insertion_point(field_set:ConnectionMessage.video_height)
}

// optional uint32 video_width = 4;
inline void ConnectionMessage::clear_video_width() {
  video_width_ = 0u;
}
inline ::google::protobuf::uint32 ConnectionMessage::video_width() const {
  // @@protoc_insertion_point(field_get:ConnectionMessage.video_width)
  return video_width_;
}
inline void ConnectionMessage::set_video_width(::google::protobuf::uint32 value) {
  
  video_width_ = value;
  // @@protoc_insertion_point(field_set:ConnectionMessage.video_width)
}

// optional uint32 depth_height = 5;
inline void ConnectionMessage::clear_depth_height() {
  depth_height_ = 0u;
}
inline ::google::protobuf::uint32 ConnectionMessage::depth_height() const {
  // @@protoc_insertion_point(field_get:ConnectionMessage.depth_height)
  return depth_height_;
}
inline void ConnectionMessage::set_depth_height(::google::protobuf::uint32 value) {
  
  depth_height_ = value;
  // @@protoc_insertion_point(field_set:ConnectionMessage.depth_height)
}

// optional uint32 depth_width = 6;
inline void ConnectionMessage::clear_depth_width() {
  depth_width_ = 0u;
}
inline ::google::protobuf::uint32 ConnectionMessage::depth_width() const {
  // @@protoc_insertion_point(field_get:ConnectionMessage.depth_width)
  return depth_width_;
}
inline void ConnectionMessage::set_depth_width(::google::protobuf::uint32 value) {
  
  depth_width_ = value;
  // @@protoc_insertion_point(field_set:ConnectionMessage.depth_width)
}

// optional uint32 message_size = 7;
inline void ConnectionMessage::clear_message_size() {
  message_size_ = 0u;
}
inline ::google::protobuf::uint32 ConnectionMessage::message_size() const {
  // @@protoc_insertion_point(field_get:ConnectionMessage.message_size)
  return message_size_;
}
inline void ConnectionMessage::set_message_size(::google::protobuf::uint32 value) {
  
  message_size_ = value;
  // @@protoc_insertion_point(field_set:ConnectionMessage.message_size)
}

#endif  // !PROTOBUF_INLINE_NOT_IN_HEADERS

// @@protoc_insertion_point(namespace_scope)

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_ConnectionMessage_2eproto__INCLUDED