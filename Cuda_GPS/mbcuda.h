// static function for handling CUDA errors
static void HandleError(cudaError_t err, const char *file, int line) 
{
	if (err != cudaSuccess) 
	{
		printf("%s in %s at line %d\n", cudaGetErrorString(err), file, line);
		exit(EXIT_FAILURE);
	}
}

// define macro for handling CUDA errors
#define HANDLE_ERROR(err) (HandleError(err, __FILE__, __LINE__))

__global__ void helper (int *f)
{
	*f = blockDim.x;
}
