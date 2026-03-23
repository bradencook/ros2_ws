from roomba import driver
import time

# sixteenth note = 12

# note sections
# song = [(36, 12), (40, 12), (43, 12), (47, 12), (48, 12), (47, 12), (43, 12), (40, 11)]

# song = [(42, 12), (46, 12), (49, 12), (53, 12), (54, 12), (53, 12), (49, 12), (46, 11)]

song = [(48, 12), (52, 12), (55, 12), (59, 12), (60, 12), (59, 12), (55, 12), (52, 11)]



def song_duration(notes):
    return sum(d for _, d in notes) / 64.0


def publish():
    driver.song(3, song)
    time.sleep(0.1)

def stranger_things():
    driver.play(3)
    time.sleep(song_duration(song) + 0.016)


if __name__ == "__main__":
    driver.startup()
    publish()
    for i in range(8):
        stranger_things()
    driver.close()
