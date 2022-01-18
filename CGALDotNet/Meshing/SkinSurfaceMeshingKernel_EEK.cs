﻿using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;

using CGALDotNet.Geometry;

namespace CGALDotNet.Meshing
{
    internal class SkinSurfaceMeshingKernel_EEK : SkinSurfaceMeshingKernel
    {
        internal override string KernelName => "EEK";

        internal static readonly SkinSurfaceMeshingKernel Instance = new SkinSurfaceMeshingKernel_EEK();

        internal override IntPtr Create()
        {
            return SkinSurfaceMeshing_EEK_Create();
        }

        internal override void Release(IntPtr ptr)
        {
            SkinSurfaceMeshing_EEK_Release(ptr);
        }

        internal override IntPtr MakeSkinSurface(double shrinkfactor, bool subdivide, HPoint3d[] points, int count)
        {
            return SkinSurfaceMeshing_EEK_MakeSkinSurface(shrinkfactor, subdivide, points, count);
        }

        [DllImport(DLL_NAME, CallingConvention = CDECL)]
        private static extern IntPtr SkinSurfaceMeshing_EEK_Create();

        [DllImport(DLL_NAME, CallingConvention = CDECL)]
        private static extern void SkinSurfaceMeshing_EEK_Release(IntPtr ptr);

        [DllImport(DLL_NAME, CallingConvention = CDECL)]
        private static extern IntPtr SkinSurfaceMeshing_EEK_MakeSkinSurface(double shrinkfactor, bool subdivide, HPoint3d[] points, int count);
    }
}