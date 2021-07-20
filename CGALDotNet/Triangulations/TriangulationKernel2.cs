﻿using System;
using System.Collections.Generic;
using System.Text;

using CGALDotNet.Geometry;
using CGALDotNet.Polygons;

namespace CGALDotNet.Triangulations
{
    internal abstract class TriangulationKernel2
    {
        internal TriangulationKernel2()
        {

        }

        internal abstract string Name { get; }

        internal abstract IntPtr Create();

        internal abstract void Release(IntPtr ptr);

        internal abstract IntPtr CreateFromPoints(Point2d[] points, int startIndex, int count);

        internal abstract IntPtr CreateFromPolygon(IntPtr ptr);

        internal abstract void Clear(IntPtr ptr);

        internal abstract bool IsValid(IntPtr ptr);

        internal abstract int VertexCount(IntPtr ptr);

        internal abstract int FaceCount(IntPtr ptr);

        internal abstract void GetPoints(IntPtr ptr, Point2d[] points, int startIndex, int count);
    }
}