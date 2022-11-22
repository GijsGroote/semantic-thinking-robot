
from sortedcontainers import SortedDict


def main():

    sd = SortedDict({0: "zero", 1: "one", 2: "two", 3: "three"})
    low = 0.5 
    high = 2.4

    result = set()
    for key in sd.irange(low, high):
        result.add(sd[key])

    print(result)


if __name__ == "__main__":
    main()
