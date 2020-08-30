#include <stdio.h>

#define		MAXLINE		10

int getLine(char s[], int lim);
void copy(char to[], char from[]);
int calcLen(char s[]);

int main()
{
	
	int len, index, row;
	char line[MAXLINE];
	char post_line[MAXLINE][MAXLINE];

	row = 0;
	while((len = getLine(line, MAXLINE)) > 0)
	{
		if(line[len-1] == '/n')	//判断读入的一行字符串的倒数第二个字符是否为'/n'
			index = len - 2;
		else
			index = len - 1;
		while(line[index] == ' ' || line[index] == '/t')   //消除字符串结尾的' '和'/t'
		{
				line[index] = line[index+1];
				line[index+1] = line[index+2];
				--index;
		}
		if(index > -1)
			copy(post_line[row++], line);	//如果line是全空格字符串，对其进行消除字符串结尾
                                                 // 处操作使得line成为一个空字符串，故不将其（空字
                                                 //符串）拷贝到post_line中。
	}

	for(index = 0; index < row; ++index)	//post_line针对其中每个字符串的倒数第二个
                                         //字符是否为'/n'在打印的时候做不同处理。
		if(post_line[index][calcLen(post_line[index])-1] != '/n')	
			printf("%s/n", post_line[index]);
		else
			printf("%s",post_line[index]);

	return 0;
}

int getLine(char s[], int lim)
{
	int i;
	char c;

	for(i = 0; i < lim-1 && (c = getchar()) != EOF && c != '/n'; ++i)
		s[i] = c;
	if(c == '/n')
	{
		s[i] = c;
		++i;
	}
	s[i] = '/0';
        fflush(stdin);			//每输入一行字符后（键入'/n'之后），清空输入缓冲区。

	return i;
}

void copy(char to[], char from[])
{
	int i;

	i = 0;
	while((to[i] = from[i]) != '/0')	
		++i;
	if(i == MAXLINE-1)
		to[i] = '/0';	
}

int calcLen(char s[])
{
	int i;

	i = 0;
	while(s[i] != '/0')
		++i;

	return i;
}
