Author: Merve Asiler

--

1. "Enter the command type:" 
	
	--> It should be either one of the followings:
		
		kergen							: Single kernel computation by KerGen
		kernel_by_cgal					: Single kernel computation by CGAL
		batch_kergen					: Batch kernel computation by KerGen
		batch_kernel_by_cgal			: Batch kernel computation by CGAL
		draw							: Draw the given mesh to the screen
		visual_comparison_of_algos		: Compare the CGAL's and KerGen's outputs.
										  CGAL/KerGen is drawn to the left/right of the screen. 

	
2. "Enter the input path (.off/.obj):" 
    
	or 

   "Enter the input folder:"

	--> Give the complete path using slash "/". (NOT backslash "\")
   
3. "Enter the output path:"

	or
	
	"Enter the output folder:

	--> It should be either one of the followings:

		The complete path of the output(s)
		d
		D
	
	"d" of "D" refers to the "default". The output(s) is created into the path where the input(s) is specified.
	The output(s) is .off files.
	
4. "Draw to the screen? (y/n):"

	--> It should be either one of the followings:
	
		y
		Y
		n
		N
		
	"y" or "Y" refers to "yes". Draws the output(s) to the screen.
	"n" or "N" refers to "no". It does not draw anything to the screen.
	

--

For bug reports or questions contact: asiler [at] ceng [dot] metu [dot] edu [dot] tr



