import os
from time import sleep

frequency = 1500
base_length = 10.0


def play_sound(sound_dur, pause_dur, count):
    if sound_dur*count == base_length:
        os.system(f'play -n synth {base_length} sin {frequency} >/dev/null 2>&1')
    elif pause_dur*count == base_length:
        sleep(base_length)
    else:
        for i in range(count):
            os.system(f'play -n synth {sound_dur} sin {frequency} >/dev/null 2>&1')
            sleep(pause_dur)


base_duration = 0.1

dist_rate_dict = {5.0: 1.0,
                  4.0: 0.8,
                  3.0: 0.6,
                  2.0: 0.5,
                  1.0: 0.4,
                  0.5: 0.2,
                  0.0: 0.1}

distances = [5.5, 4.5, 3.5, 2.1, 1.0, 0.6, 0.3]

for distance_to_traj in distances:
    for j in range(1):
        ratio = 0.1
        for i in range(len(dist_rate_dict) - 1):
            if distance_to_traj >= list(dist_rate_dict.keys())[i]:
                ratio = list(dist_rate_dict.values())[i]
                break

        sound_dur_total = ratio*base_length
        sound_count = sound_dur_total / base_duration
        sound_dur_indiv = base_duration

        pause_dur_total = base_length - sound_dur_total
        if sound_count == 0.0:
            pause_count = base_length
        else:
            pause_count = sound_count
        pause_dur_indiv = pause_dur_total / pause_count

        print(f"Playing sound for distance {distance_to_traj} with ratio {ratio}: sound_dur_ind {sound_dur_indiv}, pause_dur_ind {pause_dur_indiv}")
        print(f"Total sound dur {sound_dur_total} with count {sound_count} total pause dur {pause_dur_total} with count {pause_count}")
        play_sound(sound_dur_indiv, pause_dur_indiv, int(sound_count))
    sleep(1)

