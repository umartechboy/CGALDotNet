﻿using System;
using System.Collections.Generic;
using System.Text;

using CGALDotNet.Geometry;

namespace CGALDotNet.Arrangements
{

    public sealed class Arrangement2<K> : Arrangement2 where K : CGALKernel, new()
    {
        public Arrangement2() : base(new K())
        {

        }

        public Arrangement2(Segment2d[] points) : base(new K(), points)
        {

        }

        internal Arrangement2(IntPtr ptr) : base(new K(), ptr)
        {

        }

        public override string ToString()
        {
            return string.Format("[Arrangement2<{0}>: Vertices={1}, HalfEdges={2}, Faces={3}, Locator={4}]",
                Kernel.Name, VertexCount, HalfEdgeCount, FaceCount, Locator);
        }

        public void Assign(Arrangement2<K> other)
        {
            Kernel.Assign(Ptr, other.Ptr);
        }

        public Arrangement2<K> Copy()
        {
            var copy = new Arrangement2<K>();
            Kernel.Assign(copy.Ptr, Ptr);
            return copy;
        }

    }

    public abstract class Arrangement2 : CGALObject
    {
        private Arrangement2()
        {

        }

        internal Arrangement2(CGALKernel kernel)
        {
            Kernel = kernel.ArrangementKernel2;
            Ptr = Kernel.Create();
        }

        internal Arrangement2(CGALKernel kernel, Segment2d[] segments)
        {
            Kernel = kernel.ArrangementKernel2;
            Ptr = Kernel.CreateFromSegments(segments, 0, segments.Length);
        }

        internal Arrangement2(CGALKernel kernel, IntPtr ptr) : base(ptr)
        {
            Kernel = kernel.ArrangementKernel2;
        }

        protected private ArrangementKernel2 Kernel { get; private set; }

        public int VertexCount => Kernel.VertexCount(Ptr);

        public int IsolatedVerticesCount => Kernel.IsolatedVerticesCount(Ptr);

        public int VerticesAtInfinityCount => Kernel.VerticesAtInfinityCount(Ptr);

        public int HalfEdgeCount => Kernel.HalfEdgeCount(Ptr);

        public int EdgeCount => Kernel.EdgeCount(Ptr);

        public int FaceCount => Kernel.FaceCount(Ptr);

        public int UnboundedFaceCount => Kernel.UnboundedFaceCount(Ptr);

        public ARR_LOCATOR Locator { get; private set; }

        public bool IsEmpty => Kernel.IsEmpty(Ptr);

        public void Clear()
        {
            Kernel.Clear(Ptr);
        }

        public bool CheckIfValid()
        {
            return Kernel.IsValid(Ptr);
        }

        public void SetIndices()
        {
            SetVertexIndices();
            SetHalfEdgeIndices();
            SetFaceIndices();
        }

        public void SetVertexIndices()
        {
            Kernel.SetVertexIndices(Ptr);
        }

        public void SetHalfEdgeIndices()
        {
            Kernel.SetHalfEdgeIndices(Ptr);
        }
        public void SetFaceIndices()
        {
            Kernel.SetFaceIndices(Ptr);
        }

        public void GetPoints(Point2d[] points)
        {
            ErrorUtil.CheckBounds(points, 0, VertexCount);
            Kernel.GetPoints(Ptr, points, 0, points.Length);
        }

        public void GetSegments(Segment2d[] segments)
        {
            ErrorUtil.CheckBounds(segments, 0, EdgeCount);
            Kernel.GetSegments(Ptr, segments, 0, segments.Length);
        }

        public void GetVertices(ArrVertex2[] vertices)
        {
            ErrorUtil.CheckBounds(vertices, 0, VertexCount);
            Kernel.GetVertices(Ptr, vertices, 0, vertices.Length);
        }

        public void GetHalfEdges(ArrHalfEdge2[] edges)
        {
            ErrorUtil.CheckBounds(edges, 0, HalfEdgeCount);
            Kernel.GetHalfEdges(Ptr, edges, 0, edges.Length);
        }

        public void GetFaces(ArrFace2[] faces)
        {
            ErrorUtil.CheckBounds(faces, 0, FaceCount);
            Kernel.GetFaces(Ptr, faces, 0, faces.Length);
        }

        public void CreateLocator(ARR_LOCATOR locator)
        {
            if (Locator == locator)
                return;

            Locator = locator;
            Kernel.CreateLocator(Ptr, locator);
        }

        public void ReleaseLocator()
        {
            if (Locator == ARR_LOCATOR.NONE)
                return;

            Locator = ARR_LOCATOR.NONE;
            Kernel.ReleaseLocator(Ptr);
        }

        public bool PointQuery(Point2d point, out ArrQuery result)
        {
            return Kernel.PointQuery(Ptr, point, out result);
        }

        public bool BatchedPointQuery(Point2d[] points, ArrQuery[] results)
        {
            ErrorUtil.CheckBounds(results, 0, points.Length);
            return Kernel.BatchedPointQuery(Ptr, points, results, 0, points.Length);
        }

        public bool RayQuery(Point2d point, bool up, out ArrQuery result)
        {
            return Kernel.RayQuery(Ptr, point, up, out result);
        }

        public bool IntersectsSegment(Segment2d segment)
        {
            return Kernel.IntersectsSegment(Ptr, segment);
        }

        public void InsertPoint(Point2d point)
        {
            Kernel.InsertPoint(Ptr, point);
        }

        public bool RemoveVertex(int index)
        {
            return Kernel.RemoveVertexByIndex(Ptr, index);
        }

        public bool RemoveVertex(Point2d point)
        {
            return Kernel.RemoveVertexByPoint(Ptr, point);
        }

        public bool RemoveEdge(int index)
        {
            return Kernel.RemoveEdgeByIndex(Ptr, index);
        }

        public bool RemoveEdge(Segment2d segment)
        {
            return Kernel.RemoveEdgeBySegment(Ptr, segment);
        }

        public void Print(bool printElements = false)
        {
            var builder = new StringBuilder();
            Print(builder, printElements);
            Console.WriteLine(builder.ToString());
        }

        public void Print(StringBuilder builder, bool printElements = false)
        {
            builder.AppendLine(ToString());
            builder.AppendLine("Isolated Vertex Count = " + IsolatedVerticesCount);
            builder.AppendLine("Vertex at Infinity Count = " + VerticesAtInfinityCount);
            builder.AppendLine("Edge Count = " + EdgeCount);
            builder.AppendLine("Unbounded Face Count = " + UnboundedFaceCount);

            if(printElements)
            {
                builder.AppendLine();

                SetIndices();
                PrintVertices(builder);
                PrintHalfEdges(builder);
                PrintFaces(builder);
            }
        }

        public void PrintVertices(StringBuilder builder)
        {
            builder.AppendLine("Arrangement Vertices.\n");

            var vertices = new ArrVertex2[VertexCount];
            GetVertices(vertices);

            foreach (var v in vertices)
            {
                builder.AppendLine(v.ToString());
                builder.AppendLine("Index = " + v.Index);
                builder.AppendLine("Face Index = " + v.FaceIndex);
                builder.AppendLine("HalfEdge Index = " + v.HalfEdgeIndex);
                builder.AppendLine();
            }
        }

        public void PrintHalfEdges(StringBuilder builder)
        {
            builder.AppendLine("Arrangement Half Edges.\n");

            var edges = new ArrHalfEdge2[HalfEdgeCount];
            GetHalfEdges(edges);

            foreach (var e in edges)
            {
                builder.AppendLine(e.ToString());
                builder.AppendLine("Index = " + e.Index);
                builder.AppendLine("Source Index = " + e.SourceIndex);
                builder.AppendLine("Target Index = " + e.TargetIndex);
                builder.AppendLine("Face Index = " + e.FaceIndex);
                builder.AppendLine("Next Index = " + e.NextIndex);
                builder.AppendLine("Previous Index = " + e.PreviousIndex);
                builder.AppendLine("Twin Index = " + e.TwinIndex);
                builder.AppendLine();
            }
        }

        public void PrintFaces(StringBuilder builder)
        {
            builder.AppendLine("Arrangement Faces.\n");

            var faces = new ArrFace2[FaceCount];
            GetFaces(faces);

            foreach (var e in faces)
            {
                builder.AppendLine(e.ToString());
                builder.AppendLine("Index = " + e.Index);
                builder.AppendLine("HalfEdge Index = " + e.HalfEdgeIndex);
                builder.AppendLine();
            }
        }

        protected override void ReleasePtr()
        {
            Kernel.Release(Ptr);
        }
    }
}