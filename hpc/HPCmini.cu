#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <cuda_runtime.h>

// Huffman Node structure
struct HuffmanNode {
    unsigned char data;
    unsigned int frequency;
    HuffmanNode* left;
    HuffmanNode* right;

    HuffmanNode(unsigned char data, unsigned int frequency) : data(data), frequency(frequency), left(nullptr), right(nullptr) {}
};

// Comparison function for priority queue
struct CompareNodes {
    bool operator()(HuffmanNode* left, HuffmanNode* right) {
        return left->frequency > right->frequency;
    }
};

// Function to build Huffman tree (CPU)
HuffmanNode* buildHuffmanTree(const std::unordered_map<unsigned char, unsigned int>& frequencies) {
    std::priority_queue<HuffmanNode*, std::vector<HuffmanNode*>, CompareNodes> minHeap;

    for (const auto& pair : frequencies) {
        minHeap.push(new HuffmanNode(pair.first, pair.second));
    }

    while (minHeap.size() > 1) {
        HuffmanNode* left = minHeap.top();
        minHeap.pop();
        HuffmanNode* right = minHeap.top();
        minHeap.pop();

        HuffmanNode* parent = new HuffmanNode('\0', left->frequency + right->frequency);
        parent->left = left;
        parent->right = right;

        minHeap.push(parent);
    }

    return minHeap.top();
}

// Function to generate Huffman codes (CPU)
void generateHuffmanCodes(HuffmanNode* root, std::unordered_map<unsigned char, std::string>& huffmanCodes, std::string code = "") {
    if (root == nullptr) {
        return;
    }

    if (root->data != '\0') {
        huffmanCodes[root->data] = code;
    }

    generateHuffmanCodes(root->left, huffmanCodes, code + "0");
    generateHuffmanCodes(root->right, huffmanCodes, code + "1");
}

// CUDA kernel for frequency counting
__global__ void countFrequencies(const unsigned char* input, int inputSize, unsigned int* frequencies) {
    int tid = blockIdx.x * blockDim.x + threadIdx.x;
    if (tid < inputSize) {
        atomicAdd(&frequencies[input[tid]], 1);
    }
}

// CUDA kernel for encoding
__global__ void encodeData(const unsigned char* input, int inputSize, const unsigned int* huffmanCodeLengths, const unsigned int* huffmanCodeValues, unsigned char* output, int* outputSize) {
    int tid = blockIdx.x * blockDim.x + threadIdx.x;
    if (tid < inputSize) {
        unsigned char symbol = input[tid];
        unsigned int codeLength = huffmanCodeLengths[symbol];
        unsigned int codeValue = huffmanCodeValues[symbol];
        // In a real implementation, you'd pack the bits into the output buffer. This example skips that for simplicity.
        // This example only prints the code, not the actual encoding.
        if (threadIdx.x == 0) printf("Symbol: %c, Code: %u, Length: %u\n", symbol, codeValue, codeLength);
    }
}

int main() {
    std::string inputString = "Bubblegum";
    int inputSize = inputString.size();
    const unsigned char* input = reinterpret_cast<const unsigned char*>(inputString.c_str());

    // Allocate memory on the host
    unsigned int* hostFrequencies = new unsigned int[256]();
    unsigned char* hostOutput = new unsigned char[inputSize * 8]; // Assuming max 8 bits per character
    int hostOutputSize = 0;

    // Allocate memory on the device
    unsigned char* deviceInput;
    unsigned int* deviceFrequencies;
    unsigned char* deviceOutput;
    int* deviceOutputSize;

    cudaMalloc(&deviceInput, inputSize);
    cudaMalloc(&deviceFrequencies, 256 * sizeof(unsigned int));
    cudaMalloc(&deviceOutput, inputSize * 8);
    cudaMalloc(&deviceOutputSize, sizeof(int));

    // Copy input data to the device
    cudaMemcpy(deviceInput, input, inputSize, cudaMemcpyHostToDevice);
    cudaMemset(deviceFrequencies, 0, 256 * sizeof(unsigned int)); // Initialize frequencies to 0

    // Launch frequency counting kernel
    int blockSize = 256;
    int gridSize = (inputSize + blockSize - 1) / blockSize;
    countFrequencies<<<gridSize, blockSize>>>(deviceInput, inputSize, deviceFrequencies);

    // Copy frequencies back to the host
    cudaMemcpy(hostFrequencies, deviceFrequencies, 256 * sizeof(unsigned int), cudaMemcpyDeviceToHost);

    // Build frequency map for CPU tree building
    std::unordered_map<unsigned char, unsigned int> frequencies;
    for (int i = 0; i < 256; ++i) {
        if (hostFrequencies[i] > 0) {
            frequencies[static_cast<unsigned char>(i)] = hostFrequencies[i];
        }
    }

    // Build Huffman tree and generate codes (CPU)
    HuffmanNode* root = buildHuffmanTree(frequencies);
    std::unordered_map<unsigned char, std::string> huffmanCodes;
    generateHuffmanCodes(root, huffmanCodes);

    // Prepare code lengths and values for GPU encoding
    unsigned int hostCodeLengths[256] = { 0 };
    unsigned int hostCodeValues[256] = { 0 };
    for (const auto& pair : huffmanCodes) {
        hostCodeLengths[pair.first] = pair.second.length();
        hostCodeValues[pair.first] = std::stoul(pair.second, nullptr, 2); // Convert binary string to int
    }

    unsigned int* deviceCodeLengths;
    unsigned int* deviceCodeValues;

    cudaMalloc(&deviceCodeLengths, 256 * sizeof(unsigned int));
    cudaMalloc(&deviceCodeValues, 256 * sizeof(unsigned int));

    cudaMemcpy(deviceCodeLengths, hostCodeLengths, 256 * sizeof(unsigned int), cudaMemcpyHostToDevice);
    cudaMemcpy(deviceCodeValues, hostCodeValues, 256 * sizeof(unsigned int), cudaMemcpyHostToDevice);

    // Launch encoding kernel (example output)
    encodeData<<<gridSize, blockSize>>>(deviceInput, inputSize, deviceCodeLengths, deviceCodeValues, deviceOutput, deviceOutputSize);

    // Clean up
    cudaFree(deviceInput);
    cudaFree(deviceFrequencies);
    cudaFree(deviceOutput);
    cudaFree(deviceOutputSize);
    cudaFree(deviceCodeLengths);
    cudaFree(deviceCodeValues);
    delete[] hostFrequencies;
    delete[] hostOutput;

    return 0;
}
