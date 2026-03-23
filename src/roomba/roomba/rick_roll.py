from roomba import driver
import time

# half note = 64
# quarter note = 32
# eighth note = 16
# sixteenth note = 8


# note sections
n1 = [(67, 8), (69, 8), (72, 8), (69, 8)]  # never gonna
n2 = [(76, 24), (76, 24), (74, 48)]  # give you up/make you cry
n3 = [(74, 24), (74, 24), (72, 24), (71, 8), (69, 15)] # let you down
n4 = [(72, 32), (74, 16), (71, 24), (69, 8), (67, 32), (67, 16), (74, 32), (72, 63)]  # run around and desert you/tell a lie and hurt you
n5 = [(79, 32), (74, 16), (72, 24), (71, 8), (69, 15)]  # say goodbye

song0 = n1 + n2 + n1 + n3
song1 = n1 + n4
song2 = n1 + n2 + n1 + n5


def song_duration(notes):
    return sum(d for _, d in notes) / 64.0


def publish():
    driver.song(0, song0)
    time.sleep(0.1)
    driver.song(1, song1)
    time.sleep(0.1)
    driver.song(2, song2)
    time.sleep(0.1)


def rick_roll():
    driver.play(0)
    time.sleep(song_duration(song0) + 0.016)
    driver.play(1)
    time.sleep(song_duration(song1) + 0.016)
    driver.play(2)
    time.sleep(song_duration(song2) + 0.016)
    driver.play(1)
    time.sleep(song_duration(song1) + 0.016)


if __name__ == "__main__":
    driver.startup()
    publish()
    rick_roll()
    driver.close()