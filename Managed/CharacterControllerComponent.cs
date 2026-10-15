using System.Buffers.Binary;
using System.Globalization;
using System.Runtime.InteropServices;
using ArisenEngine.Core.ECS;
using ArisenEngine.Resources.Serialization;
using YamlDotNet.RepresentationModel;

namespace ArisenEngine.Physics;

/// <summary>
/// A character the physics runtime moves: a kinematic capsule whose base (feet) is the entity's
/// transform origin, so the visible mesh lines up without an offset. Input drives it in
/// interactive runs; bounded runs leave it standing.
/// </summary>
[StructLayout(LayoutKind.Sequential)]
public struct CharacterControllerComponent : IComponent
{
    public float Radius;
    public float HalfHeight;
    public float MoveSpeed;
}

public sealed class CharacterControllerSceneComponentCodec : ISceneComponentExtensionCodec
{
    public const uint TypeId = 1128813123; // 'CHRC'
    public const int CurrentVersion = 1;
    public const float DefaultMoveSpeed = 4.0f;

    private const int PayloadSize = 12;
    private const float MinimumExtent = 0.001f;
    private const float MaximumExtent = 10_000.0f;
    private const float MaximumMoveSpeed = 100.0f;

    public SceneComponentSchemaInfo Schema { get; } = new(
        TypeId,
        "CharacterController",
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
            RequireFields(source, ["Radius", "HalfHeight", "MoveSpeed"]);
            var value = new CharacterControllerComponent
            {
                Radius = ReadExtent(source, "Radius"),
                HalfHeight = ReadExtent(source, "HalfHeight"),
                MoveSpeed = ReadOptionalFloat(source, "MoveSpeed", DefaultMoveSpeed)
            };
            ValidateComponent(value);
            component = value;
            diagnostic = string.Empty;
            return true;
        }
        catch (Exception ex) when (ex is InvalidDataException or FormatException or OverflowException)
        {
            component = null!;
            diagnostic =
                $"[CharacterControllerSceneComponent] Scene '{context.DiagnosticPath}' entity " +
                $"'{context.EntityGuid:D}' character controller is invalid: {ex.Message}";
            return false;
        }
    }

    public byte[] WriteCooked(object component)
    {
        CharacterControllerComponent value = RequireValue(component);
        ValidateComponent(value);
        byte[] payload = new byte[PayloadSize];
        Span<byte> bytes = payload;
        BinaryPrimitives.WriteSingleLittleEndian(bytes, value.Radius);
        BinaryPrimitives.WriteSingleLittleEndian(bytes[4..], value.HalfHeight);
        BinaryPrimitives.WriteSingleLittleEndian(bytes[8..], value.MoveSpeed);
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

            var value = new CharacterControllerComponent
            {
                Radius = BinaryPrimitives.ReadSingleLittleEndian(payload),
                HalfHeight = BinaryPrimitives.ReadSingleLittleEndian(payload[4..]),
                MoveSpeed = BinaryPrimitives.ReadSingleLittleEndian(payload[8..])
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
                $"[CharacterControllerSceneComponent] Scene '{context.DiagnosticPath}' entity " +
                $"'{context.EntityGuid:D}' cooked character controller is invalid: {ex.Message}";
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

    private static void ValidateComponent(CharacterControllerComponent value)
    {
        if (!IsExtent(value.Radius) || !IsExtent(value.HalfHeight))
        {
            throw new InvalidDataException(
                "radius and half height must be finite and within their range");
        }

        if (!float.IsFinite(value.MoveSpeed) ||
            value.MoveSpeed < 0.0f ||
            value.MoveSpeed > MaximumMoveSpeed)
        {
            throw new InvalidDataException(
                $"move speed '{value.MoveSpeed}' must be finite within [0, {MaximumMoveSpeed}]");
        }
    }

    private static CharacterControllerComponent RequireValue(object component) =>
        component as CharacterControllerComponent? ??
        throw new InvalidOperationException("Character controller value has an unexpected type.");

    private static bool IsExtent(float value) =>
        float.IsFinite(value) && value >= MinimumExtent && value <= MaximumExtent;

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

    private static float ReadFloat(YamlMappingNode source, string field) =>
        float.Parse(ReadScalar(source, field), CultureInfo.InvariantCulture);

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

    private static string ReadScalar(YamlMappingNode source, string field)
    {
        foreach ((YamlNode keyNode, YamlNode value) in source.Children)
        {
            if (keyNode is YamlScalarNode key &&
                string.Equals(key.Value, field, StringComparison.Ordinal))
            {
                if (value is not YamlScalarNode scalar || string.IsNullOrWhiteSpace(scalar.Value))
                {
                    throw new InvalidDataException($"field '{field}' must be a scalar");
                }

                return scalar.Value;
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
