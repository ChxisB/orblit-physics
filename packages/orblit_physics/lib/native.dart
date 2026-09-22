/// The solver's C ABI, exactly as `orblit_physics.h` declares it.
///
/// Dart is not the only thing that will drive this solver: the same function
/// addresses can be handed to a native script or to a tool with no Dart in it.
/// That is why this is public — and why the read path is here in its raw form,
/// because a caller that already owns a float buffer wants to fill it in place
/// rather than through a copy.
///
/// Ordinary use wants `Physics` instead. Nothing here checks a handle, owns a
/// lifetime, or interprets a result.
library;

export 'src/bindings.dart';
