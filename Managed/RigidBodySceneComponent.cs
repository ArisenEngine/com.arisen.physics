using System.Buffers.Binary;
using System.Globalization;
using System.Runtime.InteropServices;
using System.Text;
using ArisenEngine.Core.ECS;
using ArisenEngine.Resources.Serialization;
using YamlDotNet.RepresentationModel;

namespace ArisenEngine.Physics;

public enum PhysicsBodyMotion
{
    Static = 0,
    Dynamic = 1
}

public enum PhysicsBodyShape
{
    Box = 0,
    Capsule = 1
}

/// <summary>
/// A body the simulation owns, authored on a scene entity. The shape is explicit rather than
/// derived from the visible mesh yet: cooked collision from authored geometry is the next slice of
/// D2, and this component is where it will land.
/// </summary>
[StructLayout(LayoutKind.Sequential)]
public struct RigidBodyComponent : IComponent
{
    public PhysicsBodyMotion Motion;
    public PhysicsBodyShape Shape;
    public float HalfExtentX;
    public float HalfExtentY;
    public float HalfExtentZ;
    public float Radius;
    public float HalfHeight;
    public float Friction;
    public float CenterX;
    public float CenterY;
    public float CenterZ;
}

/// <summary>
/// Reads, cooks, and activates the authored rigid body component. Source YAML keeps exactly the
/// fields the chosen shape needs, so a box never carries capsule parameters and vice versa.
/// </summary>
public sealed class RigidBodySceneComponentCodec : ISceneComponentExtensionCodec
{
    public const uint TypeId = 1346918722; // 'PHYB'
    public const int CurrentVersion = 1;
    public const float DefaultFriction = 0.5f;

    private const int PayloadSize = 44;
    private const float MinimumExtent = 0.001f;
    private const float MaximumExtent = 10_000.0f;
    private const float MaximumFriction = 1.0f;

    public SceneComponentSchemaInfo Schema { get; } = new(
        TypeId,
        "RigidBody",
        CurrentVersion,
        Required: false);

    public bool TryReadSource(
        in SceneComponentReadContext context,
        YamlMappingNode source,
        out object component,
        out string diagnostic)
    {
        try
        {
            PhysicsBodyMotion motion = ReadEnum<PhysicsBodyMotion>(source, "Motion");
            PhysicsBodyShape shape = ReadEnum<PhysicsBodyShape>(source, "Shape");
            float friction = ReadOptionalFloat(source, "Friction", DefaultFriction);

            var value = new RigidBodyComponent
            {
                Motion = motion,
                Shape = shape,
                Friction = friction
            };

            if (TryReadMapping(source, "Center", out YamlMappingNode? center))
            {
                RequireFields(center!, ["X", "Y", "Z"]);
                value.CenterX = ReadFiniteFloat(center!, "X");
                value.CenterY = ReadFiniteFloat(center!, "Y");
                value.CenterZ = ReadFiniteFloat(center!, "Z");
            }

            switch (shape)
            {
                case PhysicsBodyShape.Box:
                {
                    RequireFields(source, ["Motion", "Shape", "HalfExtents", "Friction", "Center"]);
                    YamlMappingNode halfExtents = ReadMapping(source, "HalfExtents");
                    RequireFields(halfExtents, ["X", "Y", "Z"]);
                    value.HalfExtentX = ReadExtent(halfExtents, "X");
                    value.HalfExtentY = ReadExtent(halfExtents, "Y");
                    value.HalfExtentZ = ReadExtent(halfExtents, "Z");
                    break;
                }
                case PhysicsBodyShape.Capsule:
                {
                    RequireFields(
                        source,
                        ["Motion", "Shape", "Radius", "HalfHeight", "Friction", "Center"]);
                    value.Radius = ReadExtent(source, "Radius");
                    value.HalfHeight = ReadExtent(source, "HalfHeight");
                    break;
                }
                default:
                {
                    throw new InvalidDataException($"unsupported shape '{shape}'");
                }
            }

            if (!IsFiniteAndRanged(friction, 0.0f, MaximumFriction))
            {
                throw new InvalidDataException(
                    $"friction '{friction.ToString(CultureInfo.InvariantCulture)}' must be finite " +
                    $"within [0, {MaximumFriction.ToString(CultureInfo.InvariantCulture)}]");
            }

            component = value;
            diagnostic = string.Empty;
            return true;
        }
        catch (Exception ex) when (ex is InvalidDataException or FormatException or OverflowException)
        {
            component = null!;
            diagnostic =
                $"[RigidBodySceneComponent] Scene '{context.DiagnosticPath}' entity " +
                $"'{context.EntityGuid:D}' rigid body is invalid: {ex.Message}";
            return false;
        }
    }

    public byte[] WriteCooked(object component)
    {
        RigidBodyComponent value = RequireValue(component);
        ValidateComponent(value);
        byte[] payload = new byte[PayloadSize];
        Span<byte> bytes = payload;
        BinaryPrimitives.WriteInt32LittleEndian(bytes, (int)value.Motion);
        BinaryPrimitives.WriteInt32LittleEndian(bytes[4..], (int)value.Shape);
        BinaryPrimitives.WriteSingleLittleEndian(bytes[8..], value.HalfExtentX);
        BinaryPrimitives.WriteSingleLittleEndian(bytes[12..], value.HalfExtentY);
        BinaryPrimitives.WriteSingleLittleEndian(bytes[16..], value.HalfExtentZ);
        BinaryPrimitives.WriteSingleLittleEndian(bytes[20..], value.Radius);
        BinaryPrimitives.WriteSingleLittleEndian(bytes[24..], value.HalfHeight);
        BinaryPrimitives.WriteSingleLittleEndian(bytes[28..], value.Friction);
        BinaryPrimitives.WriteSingleLittleEndian(bytes[32..], value.CenterX);
        BinaryPrimitives.WriteSingleLittleEndian(bytes[36..], value.CenterY);
        BinaryPrimitives.WriteSingleLittleEndian(bytes[40..], value.CenterZ);
        return payload;
    }

    public bool TryReadCooked(
        in SceneComponentReadContext context,
        ReadOnlySpan<byte> payload,
        out object component,
        out string diagnostic)
    {
        try
        {
            if (payload.Length != PayloadSize)
            {
                throw new InvalidDataException(
                    $"cooked payload length {payload.Length} does not match {PayloadSize}");
            }

            var value = new RigidBodyComponent
            {
                Motion = ReadEnumPayload<PhysicsBodyMotion>(payload, 0),
                Shape = ReadEnumPayload<PhysicsBodyShape>(payload, 4),
                HalfExtentX = BinaryPrimitives.ReadSingleLittleEndian(payload[8..]),
                HalfExtentY = BinaryPrimitives.ReadSingleLittleEndian(payload[12..]),
                HalfExtentZ = BinaryPrimitives.ReadSingleLittleEndian(payload[16..]),
                Radius = BinaryPrimitives.ReadSingleLittleEndian(payload[20..]),
                HalfHeight = BinaryPrimitives.ReadSingleLittleEndian(payload[24..]),
                Friction = BinaryPrimitives.ReadSingleLittleEndian(payload[28..]),
                CenterX = BinaryPrimitives.ReadSingleLittleEndian(payload[32..]),
                CenterY = BinaryPrimitives.ReadSingleLittleEndian(payload[36..]),
                CenterZ = BinaryPrimitives.ReadSingleLittleEndian(payload[40..])
            };
            ValidateComponent(value);
            component = value;
            diagnostic = string.Empty;
            return true;
        }
        catch (Exception ex) when (ex is InvalidDataException or OverflowException)
        {
            component = null!;
            diagnostic =
                $"[RigidBodySceneComponent] Scene '{context.DiagnosticPath}' entity " +
                $"'{context.EntityGuid:D}' cooked rigid body is invalid: {ex.Message}";
            return false;
        }
    }

    public IReadOnlyList<CookedSceneDependency> GetDependencies(object component) => [];

    public Guid GetExclusiveOwnershipId(object component) => Guid.Empty;

    public void AddToEntity(EntityManager entityManager, Entity entity, object component)
    {
        ArgumentNullException.ThrowIfNull(entityManager);
        entityManager.AddComponent(entity, RequireValue(component));
    }

    private static void ValidateComponent(RigidBodyComponent value)
    {
        if (value.Motion is not (PhysicsBodyMotion.Static or PhysicsBodyMotion.Dynamic))
        {
            throw new InvalidDataException($"unsupported motion '{value.Motion}'");
        }

        if (!IsFiniteAndRanged(value.Friction, 0.0f, MaximumFriction))
        {
            throw new InvalidDataException($"friction '{value.Friction}' is outside its range");
        }

        if (!float.IsFinite(value.CenterX) ||
            !float.IsFinite(value.CenterY) ||
            !float.IsFinite(value.CenterZ))
        {
            throw new InvalidDataException("shape center offset must be finite");
        }

        switch (value.Shape)
        {
            case PhysicsBodyShape.Box:
                if (!IsExtent(value.HalfExtentX) ||
                    !IsExtent(value.HalfExtentY) ||
                    !IsExtent(value.HalfExtentZ))
                {
                    throw new InvalidDataException("box half extents must be finite and positive");
                }

                break;
            case PhysicsBodyShape.Capsule:
                if (!IsExtent(value.Radius) || !IsExtent(value.HalfHeight))
                {
                    throw new InvalidDataException(
                        "capsule radius and half height must be finite and positive");
                }

                break;
            default:
                throw new InvalidDataException($"unsupported shape '{value.Shape}'");
        }
    }

    private static RigidBodyComponent RequireValue(object component) =>
        component as RigidBodyComponent? ??
        throw new InvalidOperationException("Rigid body component value has an unexpected type.");

    private static bool IsExtent(float value) =>
        IsFiniteAndRanged(value, MinimumExtent, MaximumExtent);

    private static bool IsFiniteAndRanged(float value, float minimum, float maximum) =>
        float.IsFinite(value) && value >= minimum && value <= maximum;

    private static TEnum ReadEnum<TEnum>(YamlMappingNode source, string field)
        where TEnum : struct, Enum
    {
        string text = ReadRequiredScalar(source, field);
        if (!Enum.TryParse(text, ignoreCase: true, out TEnum value) ||
            !Enum.IsDefined(value))
        {
            throw new InvalidDataException(
                $"field '{field}' has unsupported value '{text}'; expected " +
                $"{string.Join(", ", Enum.GetNames<TEnum>())}");
        }

        return value;
    }

    private static TEnum ReadEnumPayload<TEnum>(ReadOnlySpan<byte> payload, int offset)
        where TEnum : struct, Enum
    {
        int value = BinaryPrimitives.ReadInt32LittleEndian(payload[offset..]);
        if (!Enum.IsDefined((TEnum)Enum.ToObject(typeof(TEnum), value)))
        {
            throw new InvalidDataException($"cooked enum at offset {offset} has value {value}");
        }

        return (TEnum)Enum.ToObject(typeof(TEnum), value);
    }

    private static float ReadExtent(YamlMappingNode source, string field)
    {
        float value = ReadFloat(source, field);
        if (!IsExtent(value))
        {
            throw new InvalidDataException(
                $"field '{field}' must be finite within " +
                $"[{MinimumExtent.ToString(CultureInfo.InvariantCulture)}, " +
                $"{MaximumExtent.ToString(CultureInfo.InvariantCulture)}]");
        }

        return value;
    }

    private static float ReadOptionalFloat(YamlMappingNode source, string field, float fallback)
    {
        foreach ((YamlNode keyNode, YamlNode _) in source.Children)
        {
            if (keyNode is YamlScalarNode key &&
                string.Equals(key.Value, field, StringComparison.Ordinal))
            {
                return ReadFloat(source, field);
            }
        }

        return fallback;
    }

    private static float ReadFloat(YamlMappingNode source, string field) =>
        float.Parse(ReadRequiredScalar(source, field), CultureInfo.InvariantCulture);

    private static float ReadFiniteFloat(YamlMappingNode source, string field)
    {
        float value = ReadFloat(source, field);
        if (!float.IsFinite(value))
        {
            throw new InvalidDataException($"field '{field}' must be finite");
        }

        return value;
    }

    private static bool TryReadMapping(
        YamlMappingNode source,
        string field,
        out YamlMappingNode? mapping)
    {
        foreach ((YamlNode keyNode, YamlNode value) in source.Children)
        {
            if (keyNode is YamlScalarNode key &&
                string.Equals(key.Value, field, StringComparison.Ordinal))
            {
                mapping = value as YamlMappingNode ??
                          throw new InvalidDataException($"field '{field}' must be a mapping");
                return true;
            }
        }

        mapping = null;
        return false;
    }

    private static string ReadRequiredScalar(YamlMappingNode source, string field)
    {
        if (ReadNode(source, field) is not YamlScalarNode scalar ||
            string.IsNullOrWhiteSpace(scalar.Value))
        {
            throw new InvalidDataException($"field '{field}' is missing");
        }

        return scalar.Value;
    }

    private static YamlMappingNode ReadMapping(YamlMappingNode source, string field)
    {
        return ReadNode(source, field) as YamlMappingNode ??
               throw new InvalidDataException($"field '{field}' must be a mapping");
    }

    private static YamlNode ReadNode(YamlMappingNode mapping, string field)
    {
        foreach ((YamlNode keyNode, YamlNode value) in mapping.Children)
        {
            if (keyNode is YamlScalarNode key &&
                string.Equals(key.Value, field, StringComparison.Ordinal))
            {
                return value;
            }
        }

        throw new InvalidDataException($"required field '{field}' is missing");
    }

    private static void RequireFields(YamlMappingNode source, string[] allowed)
    {
        foreach ((YamlNode keyNode, YamlNode _) in source.Children)
        {
            if (keyNode is YamlScalarNode key &&
                !allowed.Contains(key.Value, StringComparer.Ordinal))
            {
                throw new InvalidDataException($"field '{key.Value}' is not part of the component");
            }
        }
    }
}
