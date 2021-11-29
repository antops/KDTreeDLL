//#include <iostream>
//#include "CPURayTracing.h"
//#include <vtkSTLReader.h>
//
//int main(void)
//{
//	const char* triMeshFilePath = "C:\\Users\\70743\\Desktop\\RayTracingWithKD_Tree_V2\\Mirror1.stl";
//	// const char* triMeshFilePath = "C:\\Users\\70743\\Desktop\\RayTracingWithKD_Tree_V2\\CATR.stl";
//	vtkSmartPointer<vtkSTLReader> reader =
//		vtkSmartPointer<vtkSTLReader>::New();
//	reader->SetFileName(triMeshFilePath);
//	reader->Update();
//	vtkSmartPointer<vtkPolyData> reflectorpolyData = reader->GetOutput();
//
//	CPURayTracing ct;
//	std::cout << "cores: " << ct.getPCNumCore() << " -- threads: " << ct.getPCNumThread() << std::endl;
//	ct.setSTL(reflectorpolyData);
//	//ct.printSTLs();
//	ct.generateTestRay();
//	//ct.printRays();
//	ct.buildKDTree();
//	ct.toDoRayTracing();
//	getchar();
//	return 0;
//}