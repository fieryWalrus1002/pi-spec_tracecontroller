def rise_time(r, c):
    return 2.2 * r * c * 1000000  # return in microseconds

def cval(r, t):
    return t / (2.2 * r * 1000000)  # t is in microseconds

def rval(c, t):
    return t / (2.2 * c * 1000000)  # t is in microseconds


tr = 3.9 / 10e6

print(tr)