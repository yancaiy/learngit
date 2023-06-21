#include "calterah_algo.h"

void bubble_sort(float *arr,        // data array to be sorted
                 int size,          // data array size
                 int *sorted_ind    // sorted data in terms of index in original array
                 )
{
        int i, j;
        float tmp;
        int tmp_ind;
        for (i = 0; i < size; i++) {
                sorted_ind[i] = i;
        }
        for (i = 0; i < size - 1; i++) {
                for (j = 0; j < size - i - 1; j++) {
                        if (arr[j] > arr[j+1]) {
                                tmp = arr[j];
                                arr[j] = arr[j+1];
                                arr[j+1] = tmp;

                                tmp_ind = sorted_ind[j];
                                sorted_ind[j] = sorted_ind[j+1];
                                sorted_ind[j+1] = tmp_ind;
                        }
                }
        }
}

/*fast sort*/
void quicksort(float *arr, int low, int high)
{
    if (low < high)
    {
        int i = low;
        int j = high;
        float k = arr[low];
        while (i < j)
        {
            while(i < j && arr[j] >= k)
            {
                j--;
            }
            if(i < j)
            {
                arr[i++] = arr[j];
            }
            while(i < j && arr[i] < k)
            {
                i++;
            }
            if(i < j)
            {
                arr[j--] = arr[i];
            }
        }
        arr[i] = k;
        quicksort(arr, low, i - 1);
        quicksort(arr, i + 1, high);
    }
}