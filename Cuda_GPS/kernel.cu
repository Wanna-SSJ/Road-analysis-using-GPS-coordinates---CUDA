#ifdef _MSC_VER
#define _CRT_SECURE_NO_WARNINGS
#endif

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <ctype.h>
#include <time.h>
#include <cuda.h>
#include "mbcuda.h"
#include "math_functions.h"
/*
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

__global__ void helper(int *f)
{
	*f = blockDim.x;
}
*/
struct point
{
	double coordinates[3];
};

struct line
{
	double slope, point;
};

void load_numbers(char* string, struct point &coordinate)
{
	int i = 0, j, k, l;
	double broj;

	for (j = 0; j < 3; j++)
		coordinate.coordinates[j] = 0;

	for (j = 0; j < 3; j++)
	{
		while (string[i] != '.' && isdigit(string[i]))
		{
			coordinate.coordinates[j] = coordinate.coordinates[j] * 10 + string[i] - '0';
			i++;
		}
		i++;

		k = 1;
		while (string[i] != ',' && isdigit(string[i]) && i < strlen(string))
		{
			broj = string[i] - '0';
			for (l = 0; l < k; l++)
				broj = broj / (double)10;
			coordinate.coordinates[j] = coordinate.coordinates[j] + broj;
			i++;
			k++;
		}
		i++;
	}
}

int load_data(char *file_name, struct point *field)
{
	FILE* f;
	char *string, character;
	struct point helper;
	int i = 0;

	f = fopen(file_name, "r");
	if (!f)
	{
		printf("POGRESKA PRI OTVARANJU DATOTEKE!");
		exit(1);
	}
	string = (char*)malloc(50 * sizeof(char));

	fscanf(f, "%s ", string);
	while (strncmp(string, "<altitudeMode>clampedToGround</altitudeMode>", strlen("<altitudeMode>clampedToGround</altitudeMode>")) != 0)
		fscanf(f, "%s ", string);

	fscanf(f, "%c", &character);
	while (character != '>')
		character = fgetc(f);

	fscanf(f, "%s", string);
	load_numbers(string, helper);
	field[0].coordinates[0] = helper.coordinates[0];
	field[0].coordinates[1] = helper.coordinates[1];
	field[0].coordinates[2] = helper.coordinates[2];
	i++;

	while (strncmp(string, "</coordinates>", strlen("</coordinates>")) != 0)
	{
		fscanf(f, "%s", string);
		load_numbers(string, helper);
		field[i].coordinates[0] = helper.coordinates[0];
		field[i].coordinates[1] = helper.coordinates[1];
		field[i].coordinates[2] = helper.coordinates[2];
		i++;
	}

	fclose(f);
	return i;
}

void ispis(struct point *field, int n)
{
	int i;
	for (i = 0; i < n; i++)
		printf("\n%lf,%lf,%lf", field[i].coordinates[0], field[i].coordinates[1], field[i].coordinates[2]);
}

__device__ void find_tangent(struct point a, struct point b, struct line *solution)
{
	double slope, point;
	slope = -(b.coordinates[0] - a.coordinates[0]) / (b.coordinates[1] - a.coordinates[1]);
	point = -slope * (a.coordinates[0] + b.coordinates[0]) / (double)2 + (a.coordinates[1] + b.coordinates[1]) / (double)2;
	(*solution).slope = slope;
	(*solution).point = point;
}

__device__ void find_midpoint(struct point a, struct point b, struct point *solution)
{
	(*solution).coordinates[0] = (a.coordinates[0] + b.coordinates[0]) / (double)2;
	(*solution).coordinates[1] = (a.coordinates[1] + b.coordinates[1]) / (double)2;
	(*solution).coordinates[2] = (a.coordinates[2] + b.coordinates[2]) / (double)2;
}

__device__ int find_intersection(struct line a, struct line b, struct point *solution)
{
	double x, y;
	if (abs(a.slope - b.slope) < 0.0000001)
		return -1;
	else
	{
		x = (b.point - a.point) / (a.slope - b.slope);
		y = a.slope * x + a.point;
		(*solution).coordinates[0] = x;
		(*solution).coordinates[1] = y;
		(*solution).coordinates[2] = 0;
		return 1;
	}
}

__device__ double find_distance(struct point a, struct point b)
{
	double d;
	d = sqrt((b.coordinates[0] - a.coordinates[0])*(b.coordinates[0] - a.coordinates[0]) + (b.coordinates[1] - a.coordinates[1])*(b.coordinates[1] - a.coordinates[1]));
	return d;
}

__global__ void determine_road(struct point *field, char *results, int *n)
{
	double distance1, distance2, condition;
	struct point intersection, midpoint1, midpoint2;
	struct line tangent1, tangent2;
	int flag, tid, predeccessor, successor;
	tid = blockIdx.x * blockDim.x + threadIdx.x;
	
	if (tid > *n - 1 || tid == 0)
		return;

	predeccessor = tid - 1;
	successor = tid + 1;

	find_tangent(field[predeccessor], field[tid], &tangent1);
	find_tangent(field[tid], field[successor], &tangent2);
	flag = find_intersection(tangent1, tangent2, &intersection);
	if (flag == -1)
	{
		results[tid] = 'p';
		return;
	}

	find_midpoint(field[predeccessor], field[tid], &midpoint1);
	find_midpoint(field[tid], field[successor], &midpoint2);
	distance1 = find_distance(intersection, midpoint1);
	distance2 = find_distance(intersection, midpoint2);
	condition = distance1 - distance2;

	if (abs(condition) < 0.00000001)
		results[tid] = 'r';
	else
		results[tid] = 'l';
}

void write_to_file(char* name_of_file, char *data, struct point *field, char *results, int size)
{
	FILE *f;
	int i;
	f = fopen(name_of_file, "w");

	if (!f)
	{
		printf("POGRESKA PRI OTVARANJU DATOTEKE!");
		exit(1);
	}

	fprintf(f, "Rezultati za %s", data);
	for (i = 0; i < size; i++)
		fprintf(f, "\n%lf %lf %lf %c", field[i].coordinates[0], field[i].coordinates[1], field[i].coordinates[2], results[i]);
	fclose(f);
}

int main()
{
	struct point *field, *dev_field;
	char *results, *dev_results;
	int n, *dev_n;
	clock_t begin, end;
	double time_spent;

	begin = clock();

	field = (struct point*)malloc(20000 * sizeof(struct point));
	results = (char*)malloc(20000 * sizeof(char));
	n = load_data("V3_V_S_7_11_2012.kml", field) - 1;

	HANDLE_ERROR(cudaMalloc((void**)&dev_field, 20000 * sizeof(struct point)));
	HANDLE_ERROR(cudaMalloc((void**)&dev_results, 20000 * sizeof(char)));
	HANDLE_ERROR(cudaMalloc((void**)&dev_n, sizeof(int)));
	HANDLE_ERROR(cudaMemcpy(dev_field, field, 20000 * sizeof(struct point), cudaMemcpyHostToDevice));
	HANDLE_ERROR(cudaMemcpy(dev_n, &n, sizeof(int), cudaMemcpyHostToDevice));
	
	determine_road <<<((n + 512) / 512), 512>>> (dev_field, dev_results, dev_n);

	HANDLE_ERROR(cudaMemcpy(results, dev_results, 20000 * sizeof(char), cudaMemcpyDeviceToHost));
	results[0] = 'N';
	results[n-1] = 'N';
	end = clock();
	time_spent = (double)(end - begin) / CLOCKS_PER_SEC;
	printf("DONE!");
	printf("\nTime: %lf", time_spent);
	write_to_file("V3_Records.txt", "V3_V_S_7_11_2012_V.2.kml", field, results, n);

	getchar();
	return 0;
}