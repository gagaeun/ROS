#include <stdio.h> 
#include <stdlib.h> 
#include <math.h>

void leastSquare(int arr1[], int arr2[], int n); 
float mean(int arr[], int n); 

int main(void) 
{ 
	int arr1[4] = { 2, 4, 6, 8 }; 
	int arr2[4] = { 81, 93, 91, 97 }; 
	leastSquare(arr1, arr2, 4); 
}

void leastSquare(int arr1[], int arr2[], int n) 
{ 
	float x_bar = mean(arr1, n);
	float y_bar = mean(arr2, n);

	// y = ax + b 
	float a, b; 
	float sum1 = 0, sum2 = 0;

	for (int i = 0; i < n; i++) 
	{
		sum1 += (arr1[i] - x_bar) * (arr2[i] - y_bar);
		sum2 += pow((arr1[i] - x_bar), 2); 
	}

	printf("sum1 : %lf, sum2 : %lf\n", sum1, sum2);

	a = sum1 / sum2; 
	
	b = y_bar - a * x_bar;

	printf("y = %lfx + %lf\n", a, b); 
}

float mean(int arr[], int n)
{
	float sum = 0, mean = 0; 
	
	for (int i = 0; i < n; i++)
	{ 
		sum += arr[i];
	}

	mean = sum / n; 

	printf("sum : %lf  mean : %lf\n", sum, mean);

	return mean; 
}