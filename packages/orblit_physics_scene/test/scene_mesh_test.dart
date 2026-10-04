import 'package:orblit_asset/orblit_asset.dart';
import 'package:orblit_mesh/orblit_mesh.dart' as model;
import 'package:orblit_physics_scene/orblit_physics_scene.dart';
import 'package:orblit_scene/orblit_scene.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

SceneEntity entity(
  String id,
  BodyComponent body,
  Vector3 position, {
  Vector3? scale,
}) => SceneEntity(
  id: id,
  name: id,
  components: {
    SceneComponents.body: body,
    SceneComponents.transform: TransformComponent(
      position: position,
      scale: scale,
    ),
  },
);

void main() {
  test(
    'an imported model rests on an imported mesh floor across its seam',
    () async {
      final source = MemoryAssetSource({});
      final cube = const model.Shape(kind: model.ShapeKind.cube).build();
      final floor = const model.Shape(
        kind: model.ShapeKind.plane,
        width: 20,
        depth: 20,
      ).build();
      final imported = await CollisionMesh.fromGltf(
        AssetId.parse('crate.glb'),
        cube.toGlb(),
        source,
      );
      final ground = await CollisionMesh.fromGltf(
        AssetId.parse('floor.glb'),
        floor.toGlb(),
        source,
      );
      final scene = ScenePhysics(
        SceneDocument(
          entities: [
            entity(
              'floor',
              BodyComponent(
                shape: BodyShape.mesh,
                meshVertices: ground.vertices,
                meshIndices: ground.indices,
              ),
              Vector3.zero(),
            ),
            entity(
              'crate',
              BodyComponent(shape: BodyShape.hull, hull: imported.vertices),
              Vector3(0, 4, 0),
            ),
          ],
        ),
      );
      try {
        for (var step = 0; step < 360; step++) {
          scene.advance(1 / 60);
        }
        final pose = scene.physics.transformOf(scene.bodyOf('crate')!)!;
        expect(pose[1], closeTo(0, 0.03));
        expect(scene.physics.asleep(scene.bodyOf('crate')!), isTrue);
        final saved = scene.snapshot();
        try {
          scene.apply(
            SceneDiff([RemoveEntity(scene.document['floor']!.toJson())]),
          );
          scene.restore(saved);
          expect(
            scene.physics.cast(
              from: [4, 3, 0],
              direction: [0, -1, 0],
              distance: 6,
            ),
            isNotNull,
          );
        } finally {
          saved.dispose();
        }
      } finally {
        scene.dispose();
      }
    },
  );

  test('mesh scaling, removal and restoration keep the asset bookkeeping', () {
    final body = BodyComponent(
      shape: BodyShape.mesh,
      meshVertices: [-1, 0, -1, 1, 0, -1, 0, 0, 1],
      meshIndices: [0, 2, 1],
      shapeScale: Vector3(2, 1, 1),
    );
    final scene = ScenePhysics(
      SceneDocument(
        entities: [
          entity('ramp', body, Vector3(0, 2, 0), scale: Vector3(2, 1, 1)),
        ],
      ),
    );
    try {
      expect(
        scene.physics
            .cast(from: [3, 4, -0.9], direction: [0, -1, 0], distance: 5)!
            .distance,
        closeTo(2, 0.002),
      );
      final saved = scene.snapshot();
      try {
        scene.apply(
          SceneDiff([RemoveEntity(scene.document['ramp']!.toJson())]),
        );
        expect(scene.physics.dropMesh(scene.bodyOf('ramp') ?? 1), isFalse);
        scene.restore(saved);
        expect(
          scene.physics.cast(
            from: [0, 4, 0],
            direction: [0, -1, 0],
            distance: 5,
          ),
          isNotNull,
        );
        scene.apply(
          SceneDiff([RemoveEntity(scene.document['ramp']!.toJson())]),
        );
        expect(scene.physics.stats.bodies, 0);
      } finally {
        saved.dispose();
      }
    } finally {
      scene.dispose();
    }
  });

  test('malformed meshes remain editable and inert', () {
    final scene = ScenePhysics(
      SceneDocument(
        entities: [
          entity(
            'broken',
            BodyComponent(
              shape: BodyShape.mesh,
              meshVertices: [0, 0, 0, 1, 0, 0, 0, 1, 0],
              meshIndices: [0, 1, 7],
            ),
            Vector3.zero(),
          ),
        ],
      ),
    );
    try {
      expect(scene.physics.stats.bodies, 0);
      expect(
        scene.document['broken']![SceneComponents.body],
        isA<BodyComponent>(),
      );
    } finally {
      scene.dispose();
    }
  });
}
