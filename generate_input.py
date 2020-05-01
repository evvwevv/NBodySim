import argparse
import random


def main(args):
    with open(args.filename, 'x') as f:
        for i in range(args.num_particles):
            mass = random.randrange(1e5, 1e30)
            pos = tuple(random.randrange(-1e10, 1e10) for x in range(3))
            vel = tuple(random.randrange(-1e5, 1e5) for x in range(3))
            f.write(f'{mass} ({pos[0]}, {pos[1]}, {pos[2]}) ({vel[0]}, {vel[1]}, {vel[2]})\n')


def parse_args():
    parser = argparse.ArgumentParser(description='Generate input files for sim.')
    parser.add_argument('num_particles', type=int,
                        help='Number of particles.')
    parser.add_argument('filename', type=str,
                        help='File to write to.')

    return parser.parse_args()


if __name__ == '__main__':
    main(parse_args())
