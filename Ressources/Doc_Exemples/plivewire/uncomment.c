#include <stdio.h>

int main(int argc, char *argv[]) {
  FILE *input = fopen(argv[1], "rb");
  FILE *output = fopen(argv[2], "wb");
  int skipping = 0, noskipping = 0, two = 0, five = 0;
  char temp;

  while(!feof(input)) {
    temp = getc(input);
    if(temp == '2')
      two = 1;
    if(temp == '5' && two)
      five = 1;
    if(temp == '5' && five)
      noskipping = 1;
    if(temp == '#' && !noskipping)
      skipping = 1;
    if(!skipping)
      putc(temp, output);
    if(temp == '\n')
      skipping = 0;
  }
}
