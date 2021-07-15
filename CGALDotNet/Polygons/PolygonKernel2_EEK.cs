﻿using System;
using System.Collections.Generic;
using System.Text;
using System.Runtime.InteropServices;

using CGALDotNet.Geometry;

namespace CGALDotNet.Polygons
{

    internal sealed class PolygonKernel2_EEK : PolygonKernel2
    {

        internal static readonly PolygonKernel2 Instance = new PolygonKernel2_EEK();

        internal override string Name => "EEK";

        internal override IntPtr Create()
        {
            return Polygon2_EEK_Create();
        }

        internal override IntPtr CreateFromPoints(Point2d[] points, int startIndex, int count)
        {
            return Polygon2_EEK_CreateFromPoints(points, startIndex, count);
        }

        internal override void Release(IntPtr ptr)
        {
            Polygon2_EEK_Release(ptr);
        }

        internal override int Count(IntPtr ptr)
        {
            return Polygon2_EEK_Count(ptr);
        }

        internal override IntPtr Copy(IntPtr ptr)
        {
            return Polygon2_EEK_Copy(ptr);
        }

        internal override void Clear(IntPtr ptr)
        {
            Polygon2_EEK_Copy(ptr);
        }

        internal override Point2d GetPoint(IntPtr ptr, int index)
        {
            return Polygon2_EEK_GetPoint(ptr, index);
        }

        internal override void GetPoints(IntPtr ptr, Point2d[] points, int startIndex, int count)
        {
            Polygon2_EEK_GetPoints(ptr, points, startIndex, count);
        }

        internal override void SetPoint(IntPtr ptr, int index, Point2d point)
        {
            Polygon2_EEK_SetPoint(ptr, index, point);
        }

        internal override void SetPoints(IntPtr ptr, Point2d[] points, int startIndex, int count)
        {
            Polygon2_EEK_SetPoints(ptr, points, startIndex, count);
        }

        internal override void Reverse(IntPtr ptr)
        {
            Polygon2_EEK_Reverse(ptr);
        }

        internal override bool IsSimple(IntPtr ptr)
        {
            return Polygon2_EEK_IsSimple(ptr);
        }

        internal override bool IsConvex(IntPtr ptr)
        {
            return Polygon2_EEK_IsConvex(ptr);
        }

        internal override CGAL_ORIENTATION Orientation(IntPtr ptr)
        {
            return Polygon2_EEK_Orientation(ptr);
        }

        internal override CGAL_ORIENTED_SIDE OrientedSide(IntPtr ptr, Point2d point)
        {
            return Polygon2_EEK_OrientedSide(ptr, point);
        }

        internal override double SignedArea(IntPtr ptr)
        {
            return Polygon2_EEK_SignedArea(ptr);
        }

        [DllImport("CGALWrapper.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern IntPtr Polygon2_EEK_Create();

        [DllImport("CGALWrapper.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern IntPtr Polygon2_EEK_CreateFromPoints([In] Point2d[] points, int startIndex, int count);

        [DllImport("CGALWrapper.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern void Polygon2_EEK_Release(IntPtr ptr);

        [DllImport("CGALWrapper.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern int Polygon2_EEK_Count(IntPtr ptr);

        [DllImport("CGALWrapper.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern IntPtr Polygon2_EEK_Copy(IntPtr ptr);

        [DllImport("CGALWrapper.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern void Polygon2_EEK_Clear(IntPtr ptr);

        [DllImport("CGALWrapper.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern Point2d Polygon2_EEK_GetPoint(IntPtr ptr, int index);

        [DllImport("CGALWrapper.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern void Polygon2_EEK_GetPoints(IntPtr ptr, [Out] Point2d[] points, int startIndex, int count);

        [DllImport("CGALWrapper.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern void Polygon2_EEK_SetPoint(IntPtr ptr, int index, Point2d point);

        [DllImport("CGALWrapper.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern void Polygon2_EEK_SetPoints(IntPtr ptr, [In] Point2d[] points, int startIndex, int count);

        [DllImport("CGALWrapper.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern void Polygon2_EEK_Reverse(IntPtr ptr);

        [DllImport("CGALWrapper.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern bool Polygon2_EEK_IsSimple(IntPtr ptr);

        [DllImport("CGALWrapper.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern bool Polygon2_EEK_IsConvex(IntPtr ptr);

        [DllImport("CGALWrapper.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern CGAL_ORIENTATION Polygon2_EEK_Orientation(IntPtr ptr);

        [DllImport("CGALWrapper.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern CGAL_ORIENTED_SIDE Polygon2_EEK_OrientedSide(IntPtr ptr, Point2d point);

        [DllImport("CGALWrapper.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern double Polygon2_EEK_SignedArea(IntPtr ptr);

    }
}