﻿using System;
using System.Collections.Generic;
using System.Text;
using System.Runtime.InteropServices;

using CGALDotNet.Geometry;

namespace CGALDotNet.Triangulations
{
    internal class TriangulationKernel3_EEK : TriangulationKernel3
    {
        internal override string KernelName => "EEK";

        internal static readonly TriangulationKernel3 Instance = new TriangulationKernel3_EEK();

        internal override IntPtr Create()
        {
            return Triangulation3_EEK_Create();
        }

        internal override void Release(IntPtr ptr)
        {
            Triangulation3_EEK_Release(ptr);
        }

        internal override void Clear(IntPtr ptr)
        {
            Triangulation3_EEK_Clear(ptr);
        }

        internal override IntPtr Copy(IntPtr ptr)
        {
            return Triangulation3_EEK_Copy(ptr);
        }

        internal override int Dimension(IntPtr ptr)
        {
            return Triangulation3_EEK_Dimension(ptr);
        }

        internal override bool IsValid(IntPtr ptr)
        {
            return Triangulation3_EEK_IsValid(ptr);
        }

        internal override int VertexCount(IntPtr ptr)
        {
            return Triangulation3_EEK_VertexCount(ptr);
        }

        internal override int CellCount(IntPtr ptr)
        {
            return Triangulation3_EEK_CellCount(ptr);
        }

        internal override int FiniteCellCount(IntPtr ptr)
        {
            return Triangulation3_EEK_FiniteCellCount(ptr);
        }

        internal override int EdgeCount(IntPtr ptr)
        {
            return Triangulation3_EEK_EdgeCount(ptr);
        }

        internal override int FiniteEdgeCount(IntPtr ptr)
        {
            return Triangulation3_EEK_FiniteEdgeCount(ptr);
        }

        internal override int FacetCount(IntPtr ptr)
        {
            return Triangulation3_EEK_FacetCount(ptr);
        }

        internal override int FiniteFacetCount(IntPtr ptr)
        {
            return Triangulation3_EEK_FiniteFacetCount(ptr);
        }

        internal override void InsertPoint(IntPtr ptr, Point3d point)
        {
            Triangulation3_EEK_InsertPoint(ptr, point);
        }

        internal override void InsertPoints(IntPtr ptr, Point3d[] points, int count)
        {
            Triangulation3_EEK_InsertPoints(ptr, points, count);
        }

        internal override void GetPoints(IntPtr ptr, Point3d[] points, int count)
        {
            Triangulation3_EEK_GetPoints(ptr, points, count);
        }

        internal override void GetSegmentIndices(IntPtr ptr, int[] indices, int count)
        {
            Triangulation3_EEK_GetSegmentIndices(ptr, indices, count);
        }

        internal override void GetTriangleIndices(IntPtr ptr, int[] indices, int count)
        {
            Triangulation3_EEK_GetTriangleIndices(ptr, indices, count);
        }

        internal override void GetTetrahedraIndices(IntPtr ptr, int[] indices, int count)
        {
            Triangulation3_EEK_GetTetrahedraIndices(ptr, indices, count);
        }

        internal override void Transform(IntPtr ptr, Matrix4x4d matrix)
        {
            Triangulation3_EEK_Transform(ptr, matrix);
        }

        [DllImport(DLL_NAME, CallingConvention = CDECL)]
        private static extern IntPtr Triangulation3_EEK_Create();

        [DllImport(DLL_NAME, CallingConvention = CDECL)]
        private static extern void Triangulation3_EEK_Release(IntPtr ptr);

        [DllImport(DLL_NAME, CallingConvention = CDECL)]
        private static extern void Triangulation3_EEK_Clear(IntPtr ptr);

        [DllImport(DLL_NAME, CallingConvention = CDECL)]
        private static extern IntPtr Triangulation3_EEK_Copy(IntPtr ptr);

        [DllImport(DLL_NAME, CallingConvention = CDECL)]
        private static extern int Triangulation3_EEK_Dimension(IntPtr ptr);

        [DllImport(DLL_NAME, CallingConvention = CDECL)]
        private static extern bool Triangulation3_EEK_IsValid(IntPtr ptr);

        [DllImport(DLL_NAME, CallingConvention = CDECL)]
        private static extern int Triangulation3_EEK_VertexCount(IntPtr ptr);

        [DllImport(DLL_NAME, CallingConvention = CDECL)]
        private static extern int Triangulation3_EEK_CellCount(IntPtr ptr);

        [DllImport(DLL_NAME, CallingConvention = CDECL)]
        private static extern int Triangulation3_EEK_FiniteCellCount(IntPtr ptr);

        [DllImport(DLL_NAME, CallingConvention = CDECL)]
        private static extern int Triangulation3_EEK_EdgeCount(IntPtr ptr);

        [DllImport(DLL_NAME, CallingConvention = CDECL)]
        private static extern int Triangulation3_EEK_FiniteEdgeCount(IntPtr ptr);

        [DllImport(DLL_NAME, CallingConvention = CDECL)]
        private static extern int Triangulation3_EEK_FacetCount(IntPtr ptr);

        [DllImport(DLL_NAME, CallingConvention = CDECL)]
        private static extern int Triangulation3_EEK_FiniteFacetCount(IntPtr ptr);

        [DllImport(DLL_NAME, CallingConvention = CDECL)]
        private static extern void Triangulation3_EEK_InsertPoint(IntPtr ptr, Point3d point);

        [DllImport(DLL_NAME, CallingConvention = CDECL)]
        private static extern void Triangulation3_EEK_InsertPoints(IntPtr ptr, Point3d[] points, int count);

        [DllImport(DLL_NAME, CallingConvention = CDECL)]
        private static extern void Triangulation3_EEK_GetPoints(IntPtr ptr, [Out] Point3d[] points, int count);

        [DllImport(DLL_NAME, CallingConvention = CDECL)]
        private static extern void Triangulation3_EEK_GetSegmentIndices(IntPtr ptr, [Out] int[] indices, int count);

        [DllImport(DLL_NAME, CallingConvention = CDECL)]
        private static extern void Triangulation3_EEK_GetTriangleIndices(IntPtr ptr, [Out] int[] indices, int count);

        [DllImport(DLL_NAME, CallingConvention = CDECL)]
        private static extern void Triangulation3_EEK_GetTetrahedraIndices(IntPtr ptr, [Out] int[] indices, int count);

        [DllImport(DLL_NAME, CallingConvention = CDECL)]
        private static extern void Triangulation3_EEK_Transform(IntPtr ptr, Matrix4x4d matrix);

    }
}