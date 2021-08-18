# ESkateOS
ESkateOS was made to control my custom electric skateboard, which has gone through many revisions over the years. I designed it to be a fairly robust but extensible program that can be used to safely and reliably transmit both state and throttle information over a 2.4GHz link, even with cheap radio modules (I'm using the NRF24L01+).


## Branch Structure

| Branch Name         | Board HW Version      | Remote HW Version | Notes                               |   |   |
|---------------------|-----------------------|-------------------|-------------------------------------|---|---|
| HWV1                | V1 - Aaron's original | V1-V4             |                                     |   |   |
| HWV2-ilan           | V2 - Current longboard    | V7                |                                     |   |   |
| HWV2-aaron          | V2                    | V7                | LED/headlight power switching       |   |   |
| tools               | V1/V2                 | -                 | Various tools used during testing   |   |   |
| Hardware-Board      | V1-V2                 | -                 | PCB and design files for board      |   |   |
| Hardware-Controller | -                     | V1-V7             | PCB and design files for controller |   |   |