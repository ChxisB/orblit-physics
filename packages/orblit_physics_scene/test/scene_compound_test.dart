import 'package:orblit_physics_scene/orblit_physics_scene.dart';
import 'package:orblit_scene/orblit_scene.dart';
import 'package:test/test.dart';
import 'package:vector_math/vector_math_64.dart';

SceneEntity entity(String id, BodyComponent body, {Vector3? scale}) =>
    SceneEntity(
      id: id,
      name: id,
      components: {
        SceneComponents.body: body,
        SceneComponents.transform: TransformComponent(scale: scale),
      },
    );

void main() {
  late ScenePhysics scene;
  tearDown(() => scene.dispose());

  test('compound parts survive scene files and preserve their empty gaps', () {
    final body = BodyComponent(
      shape: BodyShape.compound,
      motion: BodyMotion.fixed,
      parts: [
        BodyPart(centre: Vector3(-2, 0, 0)),
        BodyPart(centre: Vector3(2, 0, 0)),
      ],
    );
    final loaded = BodyComponent.fromJson(body.toJson());
    scene = ScenePhysics(SceneDocument(entities: [entity('arch', loaded)]));
    expect(scene.physics.overlap(at: [0, 0, 0]), isEmpty);
    expect(scene.physics.overlap(at: [2, 0, 0]), [scene.bodyOf('arch')]);
  });

  test(
    'shape scale stretches geometry separately from the scene transform',
    () {
      scene = ScenePhysics(
        SceneDocument(
          entities: [
            entity(
              'egg',
              BodyComponent(
                shape: BodyShape.sphere,
                radius: 1,
                shapeScale: Vector3(2, 1, 0.5),
                motion: BodyMotion.fixed,
              ),
              scale: Vector3(2, 1, 1),
            ),
          ],
        ),
      );
      expect(
        scene.physics
            .cast(from: [6, 0, 0], direction: [-1, 0, 0], distance: 8)!
            .distance,
        closeTo(2, 0.002),
      );
      expect(scene.physics.overlap(at: [0, 0, 0.6]), isEmpty);
      final scale =
          scene.document['egg']![SceneComponents.transform]
              as TransformComponent;
      expect(scale.scale, Vector3(2, 1, 1));
    },
  );

  test('rebuilding and removing a compound releases all its hull assets', () {
    final points = [
      for (final x in [-0.5, 0.5])
        for (final y in [-0.5, 0.5])
          for (final z in [-0.5, 0.5]) ...[x, y, z],
    ];
    final body = BodyComponent(
      shape: BodyShape.compound,
      motion: BodyMotion.fixed,
      parts: [BodyPart(shape: BodyShape.hull, hull: points)],
    );
    scene = ScenePhysics(SceneDocument(entities: [entity('rock', body)]));
    final number = scene.bodyOf('rock')!;
    final saved = scene.snapshot();
    scene.apply(SceneDiff([RemoveEntity(scene.document['rock']!.toJson())]));
    expect(scene.physics.overlap(at: [0, 0, 0]), isEmpty);
    expect(scene.physics.layHull(1, points: points), isTrue);
    scene.restore(saved);
    saved.dispose();
    expect(scene.physics.overlap(at: [0, 0, 0]), [number]);
    scene.apply(SceneDiff([RemoveEntity(scene.document['rock']!.toJson())]));
    expect(scene.physics.layHull(1, points: points), isTrue);
  });

  test('invalid parts remain editable without a world body', () {
    scene = ScenePhysics(
      SceneDocument(
        entities: [
          entity(
            'broken',
            BodyComponent(
              shape: BodyShape.compound,
              parts: [BodyPart(shape: BodyShape.plane)],
            ),
          ),
        ],
      ),
    );
    expect(scene.physics.stats.bodies, 0);
    expect(
      scene.document['broken']![SceneComponents.body],
      isA<BodyComponent>(),
    );
  });
}
