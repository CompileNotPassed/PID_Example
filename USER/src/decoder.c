#include "headfile.h"
#define DIR P35
int16 decoderRead(){
    int16 dat;
    if(DIR == 1){
        dat = -ctimer_count_read(CTIM0_P34);
    }
    else{
        dat = ctimer_count_read(CTIM0_P34);
    }
    ctimer_count_clean(CTIM0_P34);
    return dat;
}
