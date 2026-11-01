namespace Orblit.Physics.Vehicle;

/// <summary>
/// What a wheel found at the last <see cref="Vehicle.Update"/>.
/// </summary>
/// <param name="Grounded">
/// It found ground under it. A wheel that did not hangs at full droop and
/// gives no force.
/// </param>
/// <param name="Compression">
/// Metres the spring is squeezed. Past the spring's travel the wheel is pushed
/// up into the chassis and the chassis is about to rest on the floor.
/// </param>
/// <param name="Force">
/// Newtons it pushed the chassis up with. Never negative, since a spring that
/// is only held by the ground cannot pull.
/// </param>
/// <param name="Spin">Radians a second it turns at. Positive rolls the car forward.</param>
/// <param name="Steer">Radians it is turned from straight ahead. Left is positive.</param>
/// <param name="Roll">
/// Radians it has turned through, from nought up to a full turn. It grows as
/// the car rolls forward.
/// </param>
// F2: six fields, all optional and called by name.
public sealed record WheelState(
    bool Grounded = false,
    double Compression = 0.0,
    double Force = 0.0,
    double Spin = 0.0,
    double Steer = 0.0,
    double Roll = 0.0);
