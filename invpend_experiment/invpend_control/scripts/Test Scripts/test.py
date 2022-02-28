


def main():
    coordinates = []

    n = int(input("Enter number of waypoints :"))

    for i in range(0,n):
        print("Enter the waypoint",i+1)
        x = float(input("Enter the X-Coordinate :"))
        y = float(input("Enter the Y-Coordinate :"))

        coordinates.append((x,y))

    return coordinates


if __name__ == '__main__':
    val=main()
    print(val)
