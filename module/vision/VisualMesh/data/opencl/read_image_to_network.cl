#include "debayer.cl"

const sampler_t interp_sampler = CLK_NORMALIZED_COORDS_FALSE | CLK_ADDRESS_CLAMP | CLK_FILTER_LINEAR;


kernel void read_image_to_network_GRBG(read_only image2d_t image, global float2* coordinates, global float4* network) {
    const int idx = get_global_id(0);

    // Read our pixel coordinate into the image
    network[idx] = bayerToRGB(image, coordinates[idx], (float2)(1.0, 0.0));
}

kernel void read_image_to_network_RGGB(read_only image2d_t image, global float2* coordinates, global float4* network) {
    const int idx = get_global_id(0);

    // Read our pixel coordinate into the image
    network[idx] = bayerToRGB(image, coordinates[idx], (float2)(0.0, 0.0));
}

kernel void read_image_to_network_GBRG(read_only image2d_t image, global float2* coordinates, global float4* network) {
    const int idx = get_global_id(0);

    // Read our pixel coordinate into the image
    network[idx] = bayerToRGB(image, coordinates[idx], (float2)(0.0, 1.0));
}

kernel void read_image_to_network_BGGR(read_only image2d_t image, global float2* coordinates, global float4* network) {
    const int idx = get_global_id(0);

    // Read our pixel coordinate into the image
    network[idx] = bayerToRGB(image, coordinates[idx], (float2)(1.0, 1.0));
}

kernel void read_image_to_network_RGB(read_only image2d_t image, global float2* coordinates, global float4* network) {
    const int idx = get_global_id(0);

    // Read our pixel coordinate into the image
    network[idx] = read_imagef(image, interp_sampler, coordinates[idx]);
}
