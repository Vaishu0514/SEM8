#include <stdio.h>
#include <cuda_runtime.h>

__global__ void matmul(int* A, int* B, int* C) {
    int row = threadIdx.x;
    int col = threadIdx.y;

    if (row < 2 && col < 2) {
        int sum = 0;
        for (int k = 0; k < 2; k++) {
            sum += A[row * 2 + k] * B[k * 2 + col];
        }
        C[row * 2 + col] = sum;
    }
}

int main() {
    // Declare matrices A, B, and C
    int A[2][2] = {{1, 2}, {3, 4}};  // Example 2x2 matrix A
    int B[2][2] = {{5, 6}, {7, 8}};  // Example 2x2 matrix B
    int C[2][2];  // Result matrix C

    int *d_A, *d_B, *d_C;
    int size = 2 * 2 * sizeof(int); // Size of 2x2 matrix

    // Allocate memory on the device (GPU)
    cudaMalloc((void**)&d_A, size);
    cudaMalloc((void**)&d_B, size);
    cudaMalloc((void**)&d_C, size);

    // Copy data from host to device
    cudaMemcpy(d_A, A, size, cudaMemcpyHostToDevice);
    cudaMemcpy(d_B, B, size, cudaMemcpyHostToDevice);

    // Launch the kernel with a 2x2 block size (one thread per element)
    dim3 threadsPerBlock(2, 2); // 2x2 block of threads
    matmul<<<1, threadsPerBlock>>>(d_A, d_B, d_C);

    // Check for any errors during kernel launch
    cudaError_t err = cudaGetLastError();
    if (err != cudaSuccess) {
        printf("CUDA error: %s\n", cudaGetErrorString(err));
        return -1;
    }

    // Copy the result back from device to host
    cudaMemcpy(C, d_C, size, cudaMemcpyDeviceToHost);

    // Print the result matrix C
    printf("Resulting matrix C:\n");
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            printf("%d ", C[i][j]);
        }
        printf("\n");
    }

    // Free device memory
    cudaFree(d_A);
    cudaFree(d_B);
    cudaFree(d_C);

    return 0;
}
