// @author Merve Asiler

#include "KernelComputation.h"
#include "SceneManager.h"

int main()
{

	string command_type = "", draw_type = "", input_path = "", output_path = "", input_folder = "", output_folder = "";

	// Command type
	cout << "Enter the command type: ";			cin >> command_type;		cout << endl;;

	// Single Execution
	if (command_type == "draw" ||
		command_type == "kergen" ||
		command_type == "kernel_by_cgal" ||
		command_type == "visual_comparison_of_algos" ||
		command_type == "find_kernel_point_SDLP") {

		cout << "Enter the input path (.off/.obj): ";		cin >> input_path;		cout << endl;

		if (command_type == "kergen" || command_type == "kernel_by_cgal") {
			cout << "Enter the output path : ";		cin >> output_path;		cout << endl;
			
			cout << "Draw to the screen? (y/n): ";			cin >> draw_type;		cout << endl;
		}
	}

	// Batch Execution
	else if (command_type == "batch_kergen" ||
		command_type == "batch_kernel_by_cgal") {

		cout << "Enter the input folder: ";			cin >> input_folder;		cout << endl;
		cout << "Enter the output folder: ";		cin >> output_folder;		cout << endl;
		
	}

	// Undefined Operation
	else
		cout << "Undefined Operation!" << endl;

	/* Example:
		command_type = "kergen";
		draw_type = "y" or "Y" or "n" or "N";
		input_path = "D:/VS_Workspace/3D_Databases/DB-Star-shaped-meshes/data/star.off";
		output_path = "D:/VS_Workspace/3D_Databases/DB-Star-shaped-meshes/data/star" or "d";
		input_folder = "D:/VS_Workspace/3D_Databases/DB-Thingi/data";
		output_folder = "D:/VS_Workspace/3D_Databases/DB-Thingi/KernelResults-";
	*/




	cout << "Processing..." << endl << endl;

	// DRAW:
	if (command_type == "draw")
		DrawMeshToScene(input_path);

	// COMPUTE KERNEL BY <"KERGEN"  OR  "CGAL">
	else if (command_type == "kergen" || command_type == "kernel_by_cgal")
		ComputeKernel(input_path, output_path, command_type, draw_type);

	// COMPUTE KERNEL BY <"KERGEN"  OR  "CGAL"> FOR ALL MESH FILES IN THE GIVEN FOLDER
	else if (command_type == "batch_kergen" || command_type == "batch_kernel_by_cgal")
		ComputeBatchKernel(input_folder, output_folder, command_type);

	// COMPARE KERNEL RESULTS for "CGAL"  AND  "KERGEN"
	else if (command_type == "visual_comparison_of_algos")
		DoVisualComparisonOfAlgos(input_path);

	// FIND A KERNEL POINT MAXIMIZING A STATED COST FUNCTION by THIRD PARTY LIBRARY : SDLP
	else
		FindKernelPoint_SDLP(input_path);


	// Finalize
	cout << "Completed." << endl;

	return 0;

}
