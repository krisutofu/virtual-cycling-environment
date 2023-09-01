using Env3d.SumoImporter.NetFileComponents;
using Godot;
using System;
using System.Collections.Generic; // List
using System.Linq;

namespace Env3d.SumoImporter
{
	class SimpleTreeGenerator
	{
		private static readonly Mesh[] meshes = {
			ResourceLoader.Load<Mesh>("Environment/Nature/ExampleTree/Tree.mesh"),
		};

		private Node node;
		private Random rng;
		public SimpleTreeGenerator(Node parent, Random randomNumberGenerator)
		{
			this.node = parent;
			this.rng = randomNumberGenerator;
		}

		/** <summary>Adds a tree MeshInstance as child node to this generator's associated node.</summary>
		 *  <param name="treeLocation">Ground position of the tree in local coordinates of the parent node</param>
		 *  <param name="groundNormal">up-vector of the tree relative to the parent node</param>
		 *  <param name="groundRotation">rotation around the ground normal in radians</param>
		 */
		public void AddTree(Vector3 treeLocation, float? groundRotation = null, Vector3? groundNormal = null)
		{
			var treeObject = new MeshInstance();
			treeObject.Mesh = SimpleTreeGenerator.meshes[0];

			var treeDirection =  groundNormal?.Normalized() ?? Vector3.Up;
			float treeRotation = groundRotation ?? (float)( 2 * Mathf.Pi * this.rng.NextDouble() );
			this.RotateTree(treeObject, treeLocation, treeDirection, treeRotation);

			this.node.AddChild(treeObject);
		}

		private void RotateTree(MeshInstance treeObject, Vector3 treeLocation, Vector3 groundNormal, float groundRotation)
		{
			var rotation = new Quat(Vector3.Up, groundRotation);
			if (Vector3.Up.DistanceTo(groundNormal) >= 0.001)
				rotation = new Quat(Vector3.Up.Cross(groundNormal).Normalized(), Vector3.Up.AngleTo(groundNormal)) * rotation;

			treeObject.Transform = new Transform(rotation, treeLocation);
		}

		private static readonly uint numberOfTrees = 300;
		public void AddRandomTreesTo(List<NetFileEdge> edges, List<NetFileJunction> junctions)
		{
			var GetRandomFloat = () => (float)(this.rng.NextDouble() - 0.5) * 100f;

			for ( var i=1; i<=SimpleTreeGenerator.numberOfTrees; i++ )
			{
				var position = new Vector3(GetRandomFloat(), 0, GetRandomFloat());

				if (this.IsContainedInRoad(position, edges, junctions))
					continue;

				this.AddTree(position);
			}

		}

		// Note! For small sizes, this simple implementation is okay, but for growing sizes, a spatial tree structure is required
		private bool IsContainedInRoad(Vector3 position, List<NetFileEdge> edges, List<NetFileJunction> junctions)
		{
			return
					edges.Any( (street) => this.IsContainedInStreet(position, street) )
					|| junctions.Any( (junction) => this.IsContainedInJunction(position, junction) );
		}

		private bool IsContainedInStreet(Vector3 position, NetFileEdge street)
		{
			var containsPoint = (NetFileLane lane) =>
			{
				var boundary = lane.Vertices;
				var vertexCount = boundary.Length;
				var getStreetSegment = (int index) => ( boundary[ Mathf.Max(index-1, 0) ], boundary[ Mathf.Min( index+1, vertexCount-1 ) ] );

				return this.IsContainedInArea(position, vertexCount, getStreetSegment, 1f);
			};

			return street.Lanes.Any( containsPoint );			
		}

		private bool IsContainedInJunction(Vector3 position, NetFileJunction junction)
		{
			var boundary = junction.Shape;
			var vertexCount = boundary.Length;
			var getLineSegment = (int index) => ( boundary[ index ], boundary[ (index+1) % vertexCount ] );

			return this.IsContainedInArea(position, vertexCount, getLineSegment, 1f);
		}
 
		// treat position and the junction's shape as being projected on xz in 2D
		private bool IsContainedInArea(Vector3 position, int vertexCount, Func<int, (Vector3,Vector3)> getLineSegment, float tolerance = 0f)
		{
			var isInside = false;
			var toleranceSquared = tolerance * tolerance;

			for (var i=0; i < vertexCount; i++)
			{
				var (currentPoint, previousPoint) = getLineSegment(i);

				if ( toleranceSquared > 0 && toleranceSquared > ImportHelpers.ComputeDistanceToLineSegement(position, previousPoint, currentPoint).LengthSquared() )
					return true;

				if ( ImportHelpers.IsXRayIntersectingLineSegment(previousPoint, currentPoint, position) )
					isInside = !isInside;
			}
			
			return isInside;
		}
	}

}
