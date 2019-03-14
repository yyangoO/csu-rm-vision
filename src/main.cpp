/*
 * MIT License
 *
 * Copyright (c) 2019 杨洋
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
*/


#include <string>


#include <iostream>

#include "inc/includes.h"
#include "inc/parameters.h"
#include "inc/armor_monocular.h"
#include "inc/device.h"

#include "inc/rin_videocapture.h"


using namespace cv;


int main(void)
{
    RinVideoCapture mono_cap("/dev/video0", 3);
    Mat mono_img;
    params.param_init();
    mono_cap.set_format(MONO_IMAGE_X_SIZE, MONO_IMAGE_Y_SIZE, 1);
    mono_cap.set_FPS(FPS);
    mono_cap.set_exposure_time(false, 8);
    mono_cap.start_stream();
    rin_serial.serrial_cmd();
    while(1)
    {
        mono_cap >> mono_img;
        armor_mono.armor_mono_proc(mono_img, params);
        rin_serial.msg_send();
        if(waitKey(1) != -1)
        {
            break;
        }
    }
    cvDestroyAllWindows();
    return 0;
}
