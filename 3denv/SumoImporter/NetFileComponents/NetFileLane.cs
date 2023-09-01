using System;
using System.Linq;
using Godot;

namespace Env3d.SumoImporter.NetFileComponents
{
    public class NetFileLane
    {
        public string ID {get; }
        public string Allow {get; private set;}
        public string Disallow {get; private set;}
        public int Index {get; private set;}
        public float Speed {get; private set;}
        public float Length {get; private set;}
        public float Width {get; private set;}
        public Vector3[] Shape {get; private set;}

        // unfortunately only available after the Geometry was computed with the method below
        public Vector3[] Vertices { get; private set; }
        public int[] Triangles { get; private set; }
        
        public NetFileLane(string id)
        {
            this.ID = id;
        }

        public NetFileLane(laneType lane) : this(lane.id)
        {
            this.Index = int.Parse(lane.index, GameStatics.Provider);
            this.Speed = lane.speed;
            this.Length = lane.length;
            this.Width = lane.width > .1f ? lane.width : 3.2f;
            this.Allow = lane.allow;
            this.Disallow = lane.disallow;
            this.Shape = ImportHelpers.ConvertShapeString(lane.shape);
        }

        // Sometimes we only get the lane ID as a string and have to update later
        public void Update(laneType lane)
        {
            this.Index = int.Parse(lane.index, GameStatics.Provider);
            this.Speed = lane.speed;
            this.Length = lane.length;
            this.Width = lane.width > .1f ? lane.width : 3.2f;
            this.Allow = lane.allow;
            this.Disallow = lane.disallow;
            this.Shape = ImportHelpers.ConvertShapeString(lane.shape);
        }

        ///<summary>Computes the vertices and triangles of the Lane's mesh representation</summary>
        public void ComputeLaneGeometry(NetFileJunction junctionTo, NetFileJunction junctionFrom, Vector3[] laneShape, float laneWidthPadding)
        {
            this.ComputeIntersectionWithJunction(junctionFrom, laneShape.First(), out var intersectionPointStart, out var intersectionTangentStart);
            this.ComputeIntersectionWithJunction(junctionTo, laneShape.Last(), out var intersectionPointEnd, out var intersectionTangentEnd);
            var roadSegment = new LineStringRibbon( laneShape, this.Width / 2 + laneWidthPadding,  intersectionPointStart, intersectionTangentStart, intersectionPointEnd, intersectionTangentEnd);
            this.Vertices = this.Vertices = roadSegment.Vertices;
            this.Triangles = this.Triangles = roadSegment.Triangles;
        }

		/// <summary>
		/// Provides the start and end vertices of a lane to be added to the lane's shape vertices array.
		/// Aligns the end correctly with the connecting junction.
		/// </summary>
		/// <param name="junction"></param>
		/// <param name="lanePoint">vertex of the lane's lineString for which the intersection shall be computed</param>
		/// <param name="bIsStart"></param>
		/// <param name="vertices"></param>
        private void ComputeIntersectionWithJunction(NetFileJunction junction, Vector3 lanePoint, out Vector3 intersectionPoint, out Vector3 intersectionTangent)
        {
            var junctionAreaBorder = junction.Shape;
            int borderVertexCount = junctionAreaBorder.Length;

			intersectionPoint = Vector3.Zero;

			float bestDistance = float.MaxValue;
            var previousBorderPoint = junctionAreaBorder.Last();
            var bestPreviousBorderPoint = previousBorderPoint;
            var bestCurrentBorderPoint = junctionAreaBorder.First();
			for (int i = 0; i < borderVertexCount; i++)
			{
                var currentBorderPoint = junctionAreaBorder[i];
				// Christoph Note: there are multiple problems with the calculation of the junction intersection and intersection tangent if the lanePoint is not suitably aligned to the junction
				// Might create issue later.
				Vector3 testPoint;
				bool isInSegment = ImportHelpers.ClosestPointOnSegment(lanePoint, previousBorderPoint, currentBorderPoint, out testPoint);
				float distanceSq = (testPoint - lanePoint).LengthSquared();
				if (distanceSq < bestDistance)
				{
					bestPreviousBorderPoint = previousBorderPoint;
                    bestCurrentBorderPoint = currentBorderPoint;
					bestDistance = distanceSq;
					
					intersectionPoint = isInSegment ? testPoint : lanePoint;
				}
                previousBorderPoint = currentBorderPoint;
			}

			intersectionTangent = (bestPreviousBorderPoint - bestCurrentBorderPoint).Normalized();
        }
    }
}