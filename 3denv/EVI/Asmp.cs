// <auto-generated>
//     Generated by the protocol buffer compiler.  DO NOT EDIT!
//     source: asmp.proto
// </auto-generated>
#pragma warning disable 1591, 0612, 3021
#region Designer generated code

using pb = global::Google.Protobuf;
using pbc = global::Google.Protobuf.Collections;
using pbr = global::Google.Protobuf.Reflection;
using scg = global::System.Collections.Generic;
namespace Asmp {

  /// <summary>Holder for reflection information generated from asmp.proto</summary>
  public static partial class AsmpReflection {

    #region Descriptor
    /// <summary>File descriptor for asmp.proto</summary>
    public static pbr::FileDescriptor Descriptor {
      get { return descriptor; }
    }
    private static pbr::FileDescriptor descriptor;

    static AsmpReflection() {
      byte[] descriptorData = global::System.Convert.FromBase64String(
          string.Concat(
            "Cgphc21wLnByb3RvEgRhc21wGhJhc21wL3Nlc3Npb24ucHJvdG8aEmFzbXAv",
            "dmVoaWNsZS5wcm90bxoQYXNtcC9jbG91ZC5wcm90bxoXYXNtcC90cmFmZmlj",
            "bGlnaHQucHJvdG8aGGFzbXAvdmlzdWFsaXphdGlvbi5wcm90bxoYYXNtcC9o",
            "YXB0aWNzaWduYWxzLnByb3RvGhJhc21wL2hvcml6b24ucHJvdG8i6gIKB01l",
            "c3NhZ2USCgoCaWQYASABKA0SKAoHc2Vzc2lvbhhkIAEoCzIVLmFzbXAuc2Vz",
            "c2lvbi5NZXNzYWdlSAASKAoHdmVoaWNsZRhlIAEoCzIVLmFzbXAudmVoaWNs",
            "ZS5NZXNzYWdlSAASJAoFY2xvdWQYZiABKAsyEy5hc21wLmNsb3VkLk1lc3Nh",
            "Z2VIABIyCgx0cmFmZmljbGlnaHQYZyABKAsyGi5hc21wLnRyYWZmaWNsaWdo",
            "dC5NZXNzYWdlSAASNAoNdmlzdWFsaXphdGlvbhhoIAEoCzIbLmFzbXAudmlz",
            "dWFsaXphdGlvbi5NZXNzYWdlSAASNAoNaGFwdGljc2lnbmFscxhpIAEoCzIb",
            "LmFzbXAuaGFwdGljc2lnbmFscy5NZXNzYWdlSAASKAoHaG9yaXpvbhhqIAEo",
            "CzIVLmFzbXAuaG9yaXpvbi5NZXNzYWdlSABCDwoNbWVzc2FnZV9vbmVvZmIG",
            "cHJvdG8z"));
      descriptor = pbr::FileDescriptor.FromGeneratedCode(descriptorData,
          new pbr::FileDescriptor[] { global::Asmp.Session.SessionReflection.Descriptor, global::Asmp.Vehicle.VehicleReflection.Descriptor, global::Asmp.Cloud.CloudReflection.Descriptor, global::Asmp.Trafficlight.TrafficlightReflection.Descriptor, global::Asmp.Visualization.VisualizationReflection.Descriptor, global::Asmp.Hapticsignals.HapticsignalsReflection.Descriptor, global::Asmp.Horizon.HorizonReflection.Descriptor, },
          new pbr::GeneratedClrTypeInfo(null, null, new pbr::GeneratedClrTypeInfo[] {
            new pbr::GeneratedClrTypeInfo(typeof(global::Asmp.Message), global::Asmp.Message.Parser, new[]{ "Id", "Session", "Vehicle", "Cloud", "Trafficlight", "Visualization", "Hapticsignals", "Horizon" }, new[]{ "MessageOneof" }, null, null, null)
          }));
    }
    #endregion

  }
  #region Messages
  public sealed partial class Message : pb::IMessage<Message>
  #if !GOOGLE_PROTOBUF_REFSTRUCT_COMPATIBILITY_MODE
      , pb::IBufferMessage
  #endif
  {
    private static readonly pb::MessageParser<Message> _parser = new pb::MessageParser<Message>(() => new Message());
    private pb::UnknownFieldSet _unknownFields;
    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    public static pb::MessageParser<Message> Parser { get { return _parser; } }

    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    public static pbr::MessageDescriptor Descriptor {
      get { return global::Asmp.AsmpReflection.Descriptor.MessageTypes[0]; }
    }

    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    pbr::MessageDescriptor pb::IMessage.Descriptor {
      get { return Descriptor; }
    }

    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    public Message() {
      OnConstruction();
    }

    partial void OnConstruction();

    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    public Message(Message other) : this() {
      id_ = other.id_;
      switch (other.MessageOneofCase) {
        case MessageOneofOneofCase.Session:
          Session = other.Session.Clone();
          break;
        case MessageOneofOneofCase.Vehicle:
          Vehicle = other.Vehicle.Clone();
          break;
        case MessageOneofOneofCase.Cloud:
          Cloud = other.Cloud.Clone();
          break;
        case MessageOneofOneofCase.Trafficlight:
          Trafficlight = other.Trafficlight.Clone();
          break;
        case MessageOneofOneofCase.Visualization:
          Visualization = other.Visualization.Clone();
          break;
        case MessageOneofOneofCase.Hapticsignals:
          Hapticsignals = other.Hapticsignals.Clone();
          break;
        case MessageOneofOneofCase.Horizon:
          Horizon = other.Horizon.Clone();
          break;
      }

      _unknownFields = pb::UnknownFieldSet.Clone(other._unknownFields);
    }

    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    public Message Clone() {
      return new Message(this);
    }

    /// <summary>Field number for the "id" field.</summary>
    public const int IdFieldNumber = 1;
    private uint id_;
    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    public uint Id {
      get { return id_; }
      set {
        id_ = value;
      }
    }

    /// <summary>Field number for the "session" field.</summary>
    public const int SessionFieldNumber = 100;
    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    public global::Asmp.Session.Message Session {
      get { return messageOneofCase_ == MessageOneofOneofCase.Session ? (global::Asmp.Session.Message) messageOneof_ : null; }
      set {
        messageOneof_ = value;
        messageOneofCase_ = value == null ? MessageOneofOneofCase.None : MessageOneofOneofCase.Session;
      }
    }

    /// <summary>Field number for the "vehicle" field.</summary>
    public const int VehicleFieldNumber = 101;
    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    public global::Asmp.Vehicle.Message Vehicle {
      get { return messageOneofCase_ == MessageOneofOneofCase.Vehicle ? (global::Asmp.Vehicle.Message) messageOneof_ : null; }
      set {
        messageOneof_ = value;
        messageOneofCase_ = value == null ? MessageOneofOneofCase.None : MessageOneofOneofCase.Vehicle;
      }
    }

    /// <summary>Field number for the "cloud" field.</summary>
    public const int CloudFieldNumber = 102;
    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    public global::Asmp.Cloud.Message Cloud {
      get { return messageOneofCase_ == MessageOneofOneofCase.Cloud ? (global::Asmp.Cloud.Message) messageOneof_ : null; }
      set {
        messageOneof_ = value;
        messageOneofCase_ = value == null ? MessageOneofOneofCase.None : MessageOneofOneofCase.Cloud;
      }
    }

    /// <summary>Field number for the "trafficlight" field.</summary>
    public const int TrafficlightFieldNumber = 103;
    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    public global::Asmp.Trafficlight.Message Trafficlight {
      get { return messageOneofCase_ == MessageOneofOneofCase.Trafficlight ? (global::Asmp.Trafficlight.Message) messageOneof_ : null; }
      set {
        messageOneof_ = value;
        messageOneofCase_ = value == null ? MessageOneofOneofCase.None : MessageOneofOneofCase.Trafficlight;
      }
    }

    /// <summary>Field number for the "visualization" field.</summary>
    public const int VisualizationFieldNumber = 104;
    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    public global::Asmp.Visualization.Message Visualization {
      get { return messageOneofCase_ == MessageOneofOneofCase.Visualization ? (global::Asmp.Visualization.Message) messageOneof_ : null; }
      set {
        messageOneof_ = value;
        messageOneofCase_ = value == null ? MessageOneofOneofCase.None : MessageOneofOneofCase.Visualization;
      }
    }

    /// <summary>Field number for the "hapticsignals" field.</summary>
    public const int HapticsignalsFieldNumber = 105;
    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    public global::Asmp.Hapticsignals.Message Hapticsignals {
      get { return messageOneofCase_ == MessageOneofOneofCase.Hapticsignals ? (global::Asmp.Hapticsignals.Message) messageOneof_ : null; }
      set {
        messageOneof_ = value;
        messageOneofCase_ = value == null ? MessageOneofOneofCase.None : MessageOneofOneofCase.Hapticsignals;
      }
    }

    /// <summary>Field number for the "horizon" field.</summary>
    public const int HorizonFieldNumber = 106;
    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    public global::Asmp.Horizon.Message Horizon {
      get { return messageOneofCase_ == MessageOneofOneofCase.Horizon ? (global::Asmp.Horizon.Message) messageOneof_ : null; }
      set {
        messageOneof_ = value;
        messageOneofCase_ = value == null ? MessageOneofOneofCase.None : MessageOneofOneofCase.Horizon;
      }
    }

    private object messageOneof_;
    /// <summary>Enum of possible cases for the "message_oneof" oneof.</summary>
    public enum MessageOneofOneofCase {
      None = 0,
      Session = 100,
      Vehicle = 101,
      Cloud = 102,
      Trafficlight = 103,
      Visualization = 104,
      Hapticsignals = 105,
      Horizon = 106,
    }
    private MessageOneofOneofCase messageOneofCase_ = MessageOneofOneofCase.None;
    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    public MessageOneofOneofCase MessageOneofCase {
      get { return messageOneofCase_; }
    }

    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    public void ClearMessageOneof() {
      messageOneofCase_ = MessageOneofOneofCase.None;
      messageOneof_ = null;
    }

    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    public override bool Equals(object other) {
      return Equals(other as Message);
    }

    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    public bool Equals(Message other) {
      if (ReferenceEquals(other, null)) {
        return false;
      }
      if (ReferenceEquals(other, this)) {
        return true;
      }
      if (Id != other.Id) return false;
      if (!object.Equals(Session, other.Session)) return false;
      if (!object.Equals(Vehicle, other.Vehicle)) return false;
      if (!object.Equals(Cloud, other.Cloud)) return false;
      if (!object.Equals(Trafficlight, other.Trafficlight)) return false;
      if (!object.Equals(Visualization, other.Visualization)) return false;
      if (!object.Equals(Hapticsignals, other.Hapticsignals)) return false;
      if (!object.Equals(Horizon, other.Horizon)) return false;
      if (MessageOneofCase != other.MessageOneofCase) return false;
      return Equals(_unknownFields, other._unknownFields);
    }

    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    public override int GetHashCode() {
      int hash = 1;
      if (Id != 0) hash ^= Id.GetHashCode();
      if (messageOneofCase_ == MessageOneofOneofCase.Session) hash ^= Session.GetHashCode();
      if (messageOneofCase_ == MessageOneofOneofCase.Vehicle) hash ^= Vehicle.GetHashCode();
      if (messageOneofCase_ == MessageOneofOneofCase.Cloud) hash ^= Cloud.GetHashCode();
      if (messageOneofCase_ == MessageOneofOneofCase.Trafficlight) hash ^= Trafficlight.GetHashCode();
      if (messageOneofCase_ == MessageOneofOneofCase.Visualization) hash ^= Visualization.GetHashCode();
      if (messageOneofCase_ == MessageOneofOneofCase.Hapticsignals) hash ^= Hapticsignals.GetHashCode();
      if (messageOneofCase_ == MessageOneofOneofCase.Horizon) hash ^= Horizon.GetHashCode();
      hash ^= (int) messageOneofCase_;
      if (_unknownFields != null) {
        hash ^= _unknownFields.GetHashCode();
      }
      return hash;
    }

    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    public override string ToString() {
      return pb::JsonFormatter.ToDiagnosticString(this);
    }

    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    public void WriteTo(pb::CodedOutputStream output) {
    #if !GOOGLE_PROTOBUF_REFSTRUCT_COMPATIBILITY_MODE
      output.WriteRawMessage(this);
    #else
      if (Id != 0) {
        output.WriteRawTag(8);
        output.WriteUInt32(Id);
      }
      if (messageOneofCase_ == MessageOneofOneofCase.Session) {
        output.WriteRawTag(162, 6);
        output.WriteMessage(Session);
      }
      if (messageOneofCase_ == MessageOneofOneofCase.Vehicle) {
        output.WriteRawTag(170, 6);
        output.WriteMessage(Vehicle);
      }
      if (messageOneofCase_ == MessageOneofOneofCase.Cloud) {
        output.WriteRawTag(178, 6);
        output.WriteMessage(Cloud);
      }
      if (messageOneofCase_ == MessageOneofOneofCase.Trafficlight) {
        output.WriteRawTag(186, 6);
        output.WriteMessage(Trafficlight);
      }
      if (messageOneofCase_ == MessageOneofOneofCase.Visualization) {
        output.WriteRawTag(194, 6);
        output.WriteMessage(Visualization);
      }
      if (messageOneofCase_ == MessageOneofOneofCase.Hapticsignals) {
        output.WriteRawTag(202, 6);
        output.WriteMessage(Hapticsignals);
      }
      if (messageOneofCase_ == MessageOneofOneofCase.Horizon) {
        output.WriteRawTag(210, 6);
        output.WriteMessage(Horizon);
      }
      if (_unknownFields != null) {
        _unknownFields.WriteTo(output);
      }
    #endif
    }

    #if !GOOGLE_PROTOBUF_REFSTRUCT_COMPATIBILITY_MODE
    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    void pb::IBufferMessage.InternalWriteTo(ref pb::WriteContext output) {
      if (Id != 0) {
        output.WriteRawTag(8);
        output.WriteUInt32(Id);
      }
      if (messageOneofCase_ == MessageOneofOneofCase.Session) {
        output.WriteRawTag(162, 6);
        output.WriteMessage(Session);
      }
      if (messageOneofCase_ == MessageOneofOneofCase.Vehicle) {
        output.WriteRawTag(170, 6);
        output.WriteMessage(Vehicle);
      }
      if (messageOneofCase_ == MessageOneofOneofCase.Cloud) {
        output.WriteRawTag(178, 6);
        output.WriteMessage(Cloud);
      }
      if (messageOneofCase_ == MessageOneofOneofCase.Trafficlight) {
        output.WriteRawTag(186, 6);
        output.WriteMessage(Trafficlight);
      }
      if (messageOneofCase_ == MessageOneofOneofCase.Visualization) {
        output.WriteRawTag(194, 6);
        output.WriteMessage(Visualization);
      }
      if (messageOneofCase_ == MessageOneofOneofCase.Hapticsignals) {
        output.WriteRawTag(202, 6);
        output.WriteMessage(Hapticsignals);
      }
      if (messageOneofCase_ == MessageOneofOneofCase.Horizon) {
        output.WriteRawTag(210, 6);
        output.WriteMessage(Horizon);
      }
      if (_unknownFields != null) {
        _unknownFields.WriteTo(ref output);
      }
    }
    #endif

    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    public int CalculateSize() {
      int size = 0;
      if (Id != 0) {
        size += 1 + pb::CodedOutputStream.ComputeUInt32Size(Id);
      }
      if (messageOneofCase_ == MessageOneofOneofCase.Session) {
        size += 2 + pb::CodedOutputStream.ComputeMessageSize(Session);
      }
      if (messageOneofCase_ == MessageOneofOneofCase.Vehicle) {
        size += 2 + pb::CodedOutputStream.ComputeMessageSize(Vehicle);
      }
      if (messageOneofCase_ == MessageOneofOneofCase.Cloud) {
        size += 2 + pb::CodedOutputStream.ComputeMessageSize(Cloud);
      }
      if (messageOneofCase_ == MessageOneofOneofCase.Trafficlight) {
        size += 2 + pb::CodedOutputStream.ComputeMessageSize(Trafficlight);
      }
      if (messageOneofCase_ == MessageOneofOneofCase.Visualization) {
        size += 2 + pb::CodedOutputStream.ComputeMessageSize(Visualization);
      }
      if (messageOneofCase_ == MessageOneofOneofCase.Hapticsignals) {
        size += 2 + pb::CodedOutputStream.ComputeMessageSize(Hapticsignals);
      }
      if (messageOneofCase_ == MessageOneofOneofCase.Horizon) {
        size += 2 + pb::CodedOutputStream.ComputeMessageSize(Horizon);
      }
      if (_unknownFields != null) {
        size += _unknownFields.CalculateSize();
      }
      return size;
    }

    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    public void MergeFrom(Message other) {
      if (other == null) {
        return;
      }
      if (other.Id != 0) {
        Id = other.Id;
      }
      switch (other.MessageOneofCase) {
        case MessageOneofOneofCase.Session:
          if (Session == null) {
            Session = new global::Asmp.Session.Message();
          }
          Session.MergeFrom(other.Session);
          break;
        case MessageOneofOneofCase.Vehicle:
          if (Vehicle == null) {
            Vehicle = new global::Asmp.Vehicle.Message();
          }
          Vehicle.MergeFrom(other.Vehicle);
          break;
        case MessageOneofOneofCase.Cloud:
          if (Cloud == null) {
            Cloud = new global::Asmp.Cloud.Message();
          }
          Cloud.MergeFrom(other.Cloud);
          break;
        case MessageOneofOneofCase.Trafficlight:
          if (Trafficlight == null) {
            Trafficlight = new global::Asmp.Trafficlight.Message();
          }
          Trafficlight.MergeFrom(other.Trafficlight);
          break;
        case MessageOneofOneofCase.Visualization:
          if (Visualization == null) {
            Visualization = new global::Asmp.Visualization.Message();
          }
          Visualization.MergeFrom(other.Visualization);
          break;
        case MessageOneofOneofCase.Hapticsignals:
          if (Hapticsignals == null) {
            Hapticsignals = new global::Asmp.Hapticsignals.Message();
          }
          Hapticsignals.MergeFrom(other.Hapticsignals);
          break;
        case MessageOneofOneofCase.Horizon:
          if (Horizon == null) {
            Horizon = new global::Asmp.Horizon.Message();
          }
          Horizon.MergeFrom(other.Horizon);
          break;
      }

      _unknownFields = pb::UnknownFieldSet.MergeFrom(_unknownFields, other._unknownFields);
    }

    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    public void MergeFrom(pb::CodedInputStream input) {
    #if !GOOGLE_PROTOBUF_REFSTRUCT_COMPATIBILITY_MODE
      input.ReadRawMessage(this);
    #else
      uint tag;
      while ((tag = input.ReadTag()) != 0) {
        switch(tag) {
          default:
            _unknownFields = pb::UnknownFieldSet.MergeFieldFrom(_unknownFields, input);
            break;
          case 8: {
            Id = input.ReadUInt32();
            break;
          }
          case 802: {
            global::Asmp.Session.Message subBuilder = new global::Asmp.Session.Message();
            if (messageOneofCase_ == MessageOneofOneofCase.Session) {
              subBuilder.MergeFrom(Session);
            }
            input.ReadMessage(subBuilder);
            Session = subBuilder;
            break;
          }
          case 810: {
            global::Asmp.Vehicle.Message subBuilder = new global::Asmp.Vehicle.Message();
            if (messageOneofCase_ == MessageOneofOneofCase.Vehicle) {
              subBuilder.MergeFrom(Vehicle);
            }
            input.ReadMessage(subBuilder);
            Vehicle = subBuilder;
            break;
          }
          case 818: {
            global::Asmp.Cloud.Message subBuilder = new global::Asmp.Cloud.Message();
            if (messageOneofCase_ == MessageOneofOneofCase.Cloud) {
              subBuilder.MergeFrom(Cloud);
            }
            input.ReadMessage(subBuilder);
            Cloud = subBuilder;
            break;
          }
          case 826: {
            global::Asmp.Trafficlight.Message subBuilder = new global::Asmp.Trafficlight.Message();
            if (messageOneofCase_ == MessageOneofOneofCase.Trafficlight) {
              subBuilder.MergeFrom(Trafficlight);
            }
            input.ReadMessage(subBuilder);
            Trafficlight = subBuilder;
            break;
          }
          case 834: {
            global::Asmp.Visualization.Message subBuilder = new global::Asmp.Visualization.Message();
            if (messageOneofCase_ == MessageOneofOneofCase.Visualization) {
              subBuilder.MergeFrom(Visualization);
            }
            input.ReadMessage(subBuilder);
            Visualization = subBuilder;
            break;
          }
          case 842: {
            global::Asmp.Hapticsignals.Message subBuilder = new global::Asmp.Hapticsignals.Message();
            if (messageOneofCase_ == MessageOneofOneofCase.Hapticsignals) {
              subBuilder.MergeFrom(Hapticsignals);
            }
            input.ReadMessage(subBuilder);
            Hapticsignals = subBuilder;
            break;
          }
          case 850: {
            global::Asmp.Horizon.Message subBuilder = new global::Asmp.Horizon.Message();
            if (messageOneofCase_ == MessageOneofOneofCase.Horizon) {
              subBuilder.MergeFrom(Horizon);
            }
            input.ReadMessage(subBuilder);
            Horizon = subBuilder;
            break;
          }
        }
      }
    #endif
    }

    #if !GOOGLE_PROTOBUF_REFSTRUCT_COMPATIBILITY_MODE
    [global::System.Diagnostics.DebuggerNonUserCodeAttribute]
    [global::System.CodeDom.Compiler.GeneratedCode("protoc", null)]
    void pb::IBufferMessage.InternalMergeFrom(ref pb::ParseContext input) {
      uint tag;
      while ((tag = input.ReadTag()) != 0) {
        switch(tag) {
          default:
            _unknownFields = pb::UnknownFieldSet.MergeFieldFrom(_unknownFields, ref input);
            break;
          case 8: {
            Id = input.ReadUInt32();
            break;
          }
          case 802: {
            global::Asmp.Session.Message subBuilder = new global::Asmp.Session.Message();
            if (messageOneofCase_ == MessageOneofOneofCase.Session) {
              subBuilder.MergeFrom(Session);
            }
            input.ReadMessage(subBuilder);
            Session = subBuilder;
            break;
          }
          case 810: {
            global::Asmp.Vehicle.Message subBuilder = new global::Asmp.Vehicle.Message();
            if (messageOneofCase_ == MessageOneofOneofCase.Vehicle) {
              subBuilder.MergeFrom(Vehicle);
            }
            input.ReadMessage(subBuilder);
            Vehicle = subBuilder;
            break;
          }
          case 818: {
            global::Asmp.Cloud.Message subBuilder = new global::Asmp.Cloud.Message();
            if (messageOneofCase_ == MessageOneofOneofCase.Cloud) {
              subBuilder.MergeFrom(Cloud);
            }
            input.ReadMessage(subBuilder);
            Cloud = subBuilder;
            break;
          }
          case 826: {
            global::Asmp.Trafficlight.Message subBuilder = new global::Asmp.Trafficlight.Message();
            if (messageOneofCase_ == MessageOneofOneofCase.Trafficlight) {
              subBuilder.MergeFrom(Trafficlight);
            }
            input.ReadMessage(subBuilder);
            Trafficlight = subBuilder;
            break;
          }
          case 834: {
            global::Asmp.Visualization.Message subBuilder = new global::Asmp.Visualization.Message();
            if (messageOneofCase_ == MessageOneofOneofCase.Visualization) {
              subBuilder.MergeFrom(Visualization);
            }
            input.ReadMessage(subBuilder);
            Visualization = subBuilder;
            break;
          }
          case 842: {
            global::Asmp.Hapticsignals.Message subBuilder = new global::Asmp.Hapticsignals.Message();
            if (messageOneofCase_ == MessageOneofOneofCase.Hapticsignals) {
              subBuilder.MergeFrom(Hapticsignals);
            }
            input.ReadMessage(subBuilder);
            Hapticsignals = subBuilder;
            break;
          }
          case 850: {
            global::Asmp.Horizon.Message subBuilder = new global::Asmp.Horizon.Message();
            if (messageOneofCase_ == MessageOneofOneofCase.Horizon) {
              subBuilder.MergeFrom(Horizon);
            }
            input.ReadMessage(subBuilder);
            Horizon = subBuilder;
            break;
          }
        }
      }
    }
    #endif

  }

  #endregion

}

#endregion Designer generated code
