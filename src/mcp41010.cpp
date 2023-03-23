// int getMcpVal(int x){
//     // take a desired light intensity in uE, and calculate the the 16-bit value 
//     // that will be send to the MCP40101 digital potentiometer
//     // The zero_offset global variable is used to bring it to a real zero at x=0.

//     unsigned int mcpVal = 0;

//     if (x > 0){
//         mcpVal = 2E-05 * pow(x, 2) + 0.047 * x + zero_offset;
//     }
//     if (mcpVal > 255){
//         mcpVal = 255;
//     }
//     return mcpVal;
// }
