#include <stdio.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <iostream>

#define INPUT_COUNT 15
#define OUTPUT_COUNT 10

uint16_t arr_in[INPUT_COUNT] = {
    3909, 3801, 3795
    ,3691, 3581, 3475
    ,3541, 3621, 3701
    ,3693, 3501, 3683
    ,3551, 3431, 3687
};

uint16_t arr_connected[INPUT_COUNT] = {0};


uint16_t arr_out[OUTPUT_COUNT];



using namespace std;


uint8_t cell_count = 0;

void sort_numbers_top(uint16_t number[], uint8_t count) {
    uint16_t temp;
    for (uint8_t j = 0; j < count; ++j) {
        for (uint8_t k = j + 1; k < count; ++k) {
            if (number[j] > number[k]) {
                temp = number[j];
                number[j] = number[k];
                number[k] = temp;
            }
        }
    }
}

int main(int argc, const char *argv[]) {
    
    srand(time(NULL));
    for (uint8_t mm = 14; mm > 7; --mm) {
        arr_in[mm] = 500;
        for (uint8_t i = 0; i < INPUT_COUNT; ++i) { arr_connected[i] = 0; }
        for (uint8_t i = 0; i < OUTPUT_COUNT; ++i) { arr_out[i] = 0; }
        printf("================================================\n");
        cell_count = 0;
        for (uint8_t i = 0; i <  INPUT_COUNT; ++i) {
            if (arr_in[i] > 512) {
                arr_in[i] += int16_t(256 - rand() % 512);
                //printf("%u ", arr_in[i]);
                arr_connected[cell_count++] = arr_in[i];
            }
        }
        
        sort_numbers_top(arr_connected, cell_count);
        
        printf("\n %d connected, sorted:\n", cell_count+1);
        for (uint8_t i = 0; i <  cell_count; ++i) printf("%u ", arr_connected[i]);
        printf("\n");
        
        if (cell_count > OUTPUT_COUNT) {
            
            int8_t b = OUTPUT_COUNT - 1;
            for (uint8_t a = 0; a < OUTPUT_COUNT/2; ++a) {
                arr_out[a] = arr_connected[a];
                arr_out[b--] = arr_connected[cell_count-a-1];
            }
        } else if (OUTPUT_COUNT > cell_count) {
            uint16_t summ = 0;
            for (uint8_t i = 0; i < cell_count; ++i) { summ += arr_connected[i]; }
            summ /= cell_count;
            for (uint8_t i = 0; i < OUTPUT_COUNT; ++i) { arr_out[i] = summ; }
            for (uint8_t i = 0; i < cell_count/2; ++i) { arr_out[i] = arr_connected[i]; }
            uint8_t x = OUTPUT_COUNT - 1;
            for (uint8_t i = cell_count-1; i > cell_count/2; --i) {
                arr_out[x--] = arr_connected[i];
            }
        } else {
            for (uint8_t i = 0; i < cell_count; ++i) { arr_out[i] = arr_connected[i]; }
        }
        
        printf("\nC in ascending order:\n");
        for (uint8_t i = 0; i <  OUTPUT_COUNT; ++i) printf("%u ", arr_out[i]);
        printf("\n");
    }
}
