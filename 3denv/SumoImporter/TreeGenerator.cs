using Godot;
using System;
using System.CodeDom;
using static Godot.GeometryInstance;

namespace Env3d.SumoImporter {

    class SimpleTreeGenerator {

        private static Mesh[] meshes = {
            ResourceLoader.Load<Mesh>("Environment/Nature/ExampleTree/Tree.mesh"),
        };

        public static void AddRandomTreesTo(Node node) {

            var treeObject = new MeshInstance();
            treeObject.Mesh = meshes[0];

            var treeOrientation = new Quat(new Vector3(0f,0f,0f));
            var treeLocation = new Vector3(0f,0f,0f);

            treeObject.Transform = new Transform(treeOrientation, treeLocation);

            node.AddChild(treeObject);
        }
    }

}