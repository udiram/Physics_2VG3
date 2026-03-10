#!/usr/bin/python3

def match_ends(words):
    count = 0  # initalize counter
    for word in words:  # iterate through each word in the list
        if len(word) >= 2 and word[0] == word[-1]:  # first condition: length >= 2, second condition: first and last char are the same
            count += 1  # increment counter if both conditions are met
    return count  # return the final count


def front_x(words):
    x, no_x = [], []  # create two empty lists
    for word in words:  # iterate through each word in the list
        if word.startswith('x'):  # check if the word starts with 'x'
            x.append(word)  # add to x list
        else:  # if it doesn't start with 'x'
            no_x.append(word)  # add to no_x list
    x.sort()  # sort both lists
    no_x.sort()
    combinedlist = x + no_x  # concatenate the two lists
    return combinedlist  # return the combined list


def sort_last(tuples):
    sortedlist = sorted(tuples, key=lambda t: t[-1])  # sort the list of tuples based on the last element of each tuple
    return sortedlist


def test(got, expected):
  if got == expected:
    prefix = ' OK '
  else:
    prefix = '  X '
  print('%s got: %s expected: %s' % (prefix, repr(got), repr(expected)))


# Calls the above functions with interesting inputs.
def main():
  print('match_ends')
  test(match_ends(['aba', 'xyz', 'aa', 'x', 'bbb']), 3)
  test(match_ends(['', 'x', 'xy', 'xyx', 'xx']), 2)
  test(match_ends(['aaa', 'be', 'abc', 'hello']), 1)

  print()
  print('front_x')
  test(front_x(['bbb', 'ccc', 'axx', 'xzz', 'xaa']),
       ['xaa', 'xzz', 'axx', 'bbb', 'ccc'])
  test(front_x(['ccc', 'bbb', 'aaa', 'xcc', 'xaa']),
       ['xaa', 'xcc', 'aaa', 'bbb', 'ccc'])
  test(front_x(['mix', 'xyz', 'apple', 'xanadu', 'aardvark']),
       ['xanadu', 'xyz', 'aardvark', 'apple', 'mix'])


  print()
  print('sort_last')
  test(sort_last([(1, 3), (3, 2), (2, 1)]),
       [(2, 1), (3, 2), (1, 3)])
  test(sort_last([(2, 3), (1, 2), (3, 1)]),
       [(3, 1), (1, 2), (2, 3)])
  test(sort_last([(1, 7), (1, 3), (3, 4, 5), (2, 2)]),
       [(2, 2), (1, 3), (3, 4, 5), (1, 7)])


if __name__ == '__main__':
  main()
