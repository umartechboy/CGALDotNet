﻿using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Text;

using CGALDotNetGeometry.Numerics;

namespace CGALDotNet.Triangulations
{
    internal abstract class BaseTriangulationKernel3 : CGALObjectKernel
    {

        internal abstract IntPtr Create();

        internal abstract void Release(IntPtr ptr);

        internal abstract void Clear(IntPtr ptr);

        internal abstract IntPtr Copy(IntPtr ptr);

        internal abstract int BuildStamp(IntPtr ptr);

        internal abstract int Dimension(IntPtr ptr);

        internal abstract bool IsValid(IntPtr ptr);

        internal abstract int VertexCount(IntPtr ptr);

        internal abstract int CellCount(IntPtr ptr);

        internal abstract int FiniteCellCount(IntPtr ptr);

        internal abstract int EdgeCount(IntPtr ptr);

        internal abstract int FiniteEdgeCount(IntPtr ptr);

        internal abstract int FacetCount(IntPtr ptr);

        internal abstract int FiniteFacetCount(IntPtr ptr);

        internal abstract void InsertPoint(IntPtr ptr, Point3d point);

	    internal abstract void InsertPoints(IntPtr ptr, Point3d[] points, int count);

        internal abstract void GetPoints(IntPtr ptr, Point3d[] points, int count);

        internal abstract void GetVertices(IntPtr ptr, TriVertex3[] vertices, int count);

        internal abstract bool GetVertex(IntPtr ptr, int index, out TriVertex3 vertex);

        internal abstract void GetCells(IntPtr ptr, TriCell3[] cells, int count);

        internal abstract bool GetCell(IntPtr ptr, int index, out TriCell3 cell);

        internal abstract void GetSegmentIndices(IntPtr ptr, int[] indices, int count);

        internal abstract void GetTriangleIndices(IntPtr ptr, int[] indices, int count);

        internal abstract void GetTetrahedraIndices(IntPtr ptr, int[] indices, int count);

        internal abstract void Transform(IntPtr ptr, Matrix4x4d matrix);
    }
}