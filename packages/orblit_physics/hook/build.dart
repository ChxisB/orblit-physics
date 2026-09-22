import 'package:code_assets/code_assets.dart';
import 'package:hooks/hooks.dart';
import 'package:native_toolchain_c/native_toolchain_c.dart';

/// Builds the solver into one library the Dart bindings open over FFI.
void main(List<String> args) async {
  await build(args, (input, output) async {
    if (!input.config.buildCodeAssets) return;

    final root = input.packageRoot;
    final builder = CBuilder.library(
      name: 'orblit_physics',
      assetName: 'orblit_physics',
      sources: [
        root.resolve('src/orblit_physics.cpp').toFilePath(),
        root.resolve('src/world.cpp').toFilePath(),
        root.resolve('src/collide.cpp').toFilePath(),
        root.resolve('src/cast.cpp').toFilePath(),
        root.resolve('src/solver.cpp').toFilePath(),
      ],
      includes: [
        root.resolve('include/').toFilePath(),
        root.resolve('src/').toFilePath(),
      ],
      language: Language.cpp,
    );

    await builder.run(input: input, output: output, logger: null);
  });
}
