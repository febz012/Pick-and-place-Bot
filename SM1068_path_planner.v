module SM1068_path_planner(
input clk, input [1:0] orientation,

output [1:0] sl0,output [1:0] sl1,output [1:0] sl2,output [1:0] sl3,output [1:0] sl4,output [1:0] sl5,output [1:0] sl6,output [1:0] sl7,output [1:0] sl8,output [1:0] sl9,
output [1:0] sl10,output [1:0] sl11,output [1:0] sl12,output [1:0] sl13,output [1:0] sl14,output [1:0] sl15,output [1:0] sl16,output [1:0] sl17,output [1:0] sl18,output [1:0] sl19,
output [1:0] sl20,output [1:0] sl21,output [1:0] sl22,output [1:0] sl23,output [1:0] sl24,output [1:0] sl25,output [1:0] sl26,output [1:0] sl27,output [1:0] sl28,output [1:0] sl29,
output [1:0] sl30,output [1:0] sl31,output [1:0] sl32,output [1:0] sl33,output [1:0] sl34,output [1:0] sl35,output [1:0] sl36,input enpl,input [5:0] iend,input [5:0] istr, output[3:0] len
);

parameter NEAR=0,ADJ=1,PATH=2,DIR=3,GETDIR=4,ASGDIR=5,PLANDIR=6,STOP=7,REINIT=8;

reg [5:0] mapbot [36:0][36:0];
reg [5:0] noddir [36:0][36:0];
reg [36:0] vis;
reg [10:0] dista[36:0];
reg [10:0] min;
reg [5:0] i;
reg [5:0] k;
reg [5:0] j;
reg [5:0] le;

reg [6:0] dcount=0;
reg [3:0] length=0;

reg [5:0] stno;
reg [5:0] enno;  
reg [5:0] nearest;
reg [3:0] state=NEAR;
reg [5:0] parent [36:0];
reg [5:0] path [36:0];

reg [5:0] curnod;
reg [5:0] nxtnod;

reg [1:0] plan[36:0];

reg [5:0] temp;

reg [5:0] curdir;
reg [5:0] nxtdir;


always@(posedge clk) begin
enno<=iend;
stno<=istr;
end

initial begin
   stno = 15;
	enno = 28;
   temp = 28;	
	vis = 0;
	curdir=2;
	
	i=0;
	j=0;
	k=0;
	le=0;
	
	min=55;
	mapbot[0][0] = 0;mapbot[0][1] = 3;mapbot[0][2]=55;mapbot[0][3]=55;mapbot[0][4]=55;mapbot[0][5]=55;mapbot[0][6]=55;mapbot[0][7]=1;mapbot[0][8]=2;mapbot[0][9]=55;mapbot[0][10]=55;mapbot[0][11]=55;mapbot[0][12]=55;mapbot[0][13]=55;mapbot[0][14]=55;mapbot[0][15]=55;mapbot[0][16]=55;mapbot[0][17]=55;mapbot[0][18]=55;mapbot[0][19]=55;mapbot[0][20]=55;mapbot[0][21]=55;mapbot[0][22]=55;mapbot[0][23]=55;mapbot[0][24]=55;mapbot[0][25]=55;mapbot[0][26]=55;mapbot[0][27]=55;mapbot[0][28]=55;mapbot[0][29]=55;mapbot[0][30]=55;mapbot[0][31]=55;mapbot[0][32]=55;mapbot[0][33]=55;mapbot[0][34]=55;mapbot[0][35]=55;mapbot[0][36]=55;
	mapbot[1][0] =3;mapbot[1][1] = 0;mapbot[1][2]=55;mapbot[1][3]=55;mapbot[1][4]=55;mapbot[1][5]=2;mapbot[1][6]=55;mapbot[1][7]=55;mapbot[1][8]=55;mapbot[1][9]=55;mapbot[1][10]=55;mapbot[1][11]=55;mapbot[1][12]=55;mapbot[1][13]=55;mapbot[1][14]=55;mapbot[1][15]=6;mapbot[1][16]=55;mapbot[1][17]=55;mapbot[1][18]=55;mapbot[1][19]=55;mapbot[1][20]=55;mapbot[1][21]=55;mapbot[1][22]=55;mapbot[1][23]=55;mapbot[1][24]=55;mapbot[1][25]=55;mapbot[1][26]=55;mapbot[1][27]=55;mapbot[1][28]=55;mapbot[1][29]=55;mapbot[1][30]=55;mapbot[1][31]=55;mapbot[1][32]=55;mapbot[1][33]=55;mapbot[1][34]=55;mapbot[1][35]=55;mapbot[1][36]=55;
	mapbot[2][0] =55;mapbot[2][1] = 55;mapbot[2][2]=0;mapbot[2][3]=55;mapbot[2][4]=55;mapbot[2][5]=55;mapbot[2][6]=55;mapbot[2][7]=55;mapbot[2][8]=55;mapbot[2][9]=55;mapbot[2][10]=55;mapbot[2][11]=55;mapbot[2][12]=55;mapbot[2][13]=55;mapbot[2][14]=1;mapbot[2][15]=55;mapbot[2][16]=55;mapbot[2][17]=55;mapbot[2][18]=55;mapbot[2][19]=55;mapbot[2][20]=55;mapbot[2][21]=55;mapbot[2][22]=55;mapbot[2][23]=55;mapbot[2][24]=55;mapbot[2][25]=55;mapbot[2][26]=55;mapbot[2][27]=55;mapbot[2][28]=55;mapbot[2][29]=55;mapbot[2][30]=55;mapbot[2][31]=55;mapbot[2][32]=55;mapbot[2][33]=55;mapbot[2][34]=55;mapbot[2][35]=55;mapbot[2][36]=55;
	mapbot[3][0] =55;mapbot[3][1] = 55;mapbot[3][2]=55;mapbot[3][3]=0;mapbot[3][4]=55;mapbot[3][5]=55;mapbot[3][6]=55;mapbot[3][7]=55;mapbot[3][8]=55;mapbot[3][9]=55;mapbot[3][10]=55;mapbot[3][11]=55;mapbot[3][12]=55;mapbot[3][13]=1;mapbot[3][14]=55;mapbot[3][15]=55;mapbot[3][16]=55;mapbot[3][17]=55;mapbot[3][18]=55;mapbot[3][19]=55;mapbot[3][20]=55;mapbot[3][21]=55;mapbot[3][22]=55;mapbot[3][23]=55;mapbot[3][24]=55;mapbot[3][25]=55;mapbot[3][26]=55;mapbot[3][27]=55;mapbot[3][28]=55;mapbot[3][29]=55;mapbot[3][30]=55;mapbot[3][31]=55;mapbot[3][32]=55;mapbot[3][33]=55;mapbot[3][34]=55;mapbot[3][35]=55;mapbot[3][36]=55;
	mapbot[4][0] =55;mapbot[4][1] = 55;mapbot[4][2]=55;mapbot[4][3]=55;mapbot[4][4]=0;mapbot[4][5]=55;mapbot[4][6]=55;mapbot[4][7]=55;mapbot[4][8]=55;mapbot[4][9]=55;mapbot[4][10]=55;mapbot[4][11]=55;mapbot[4][12]=1;mapbot[4][13]=55;mapbot[4][14]=55;mapbot[4][15]=55;mapbot[4][16]=55;mapbot[4][17]=55;mapbot[4][18]=55;mapbot[4][19]=55;mapbot[4][20]=55;mapbot[4][21]=55;mapbot[4][22]=55;mapbot[4][23]=55;mapbot[4][24]=55;mapbot[4][25]=55;mapbot[4][26]=55;mapbot[4][27]=55;mapbot[4][28]=55;mapbot[4][29]=55;mapbot[4][30]=55;mapbot[4][31]=55;mapbot[4][32]=55;mapbot[4][33]=55;mapbot[4][34]=55;mapbot[4][35]=55;mapbot[4][36]=55;
	mapbot[5][0] =55;mapbot[5][1] = 2;mapbot[5][2]=55;mapbot[5][3]=55;mapbot[5][4]=55;mapbot[5][5]=0;mapbot[5][6]=1;mapbot[5][7]=55;mapbot[5][8]=55;mapbot[5][9]=55;mapbot[5][10]=1;mapbot[5][11]=55;mapbot[5][12]=55;mapbot[5][13]=55;mapbot[5][14]=55;mapbot[5][15]=55;mapbot[5][16]=55;mapbot[5][17]=55;mapbot[5][18]=55;mapbot[5][19]=55;mapbot[5][20]=55;mapbot[5][21]=55;mapbot[5][22]=55;mapbot[5][23]=55;mapbot[5][24]=55;mapbot[5][25]=55;mapbot[5][26]=55;mapbot[5][27]=55;mapbot[5][28]=55;mapbot[5][29]=55;mapbot[5][30]=55;mapbot[5][31]=55;mapbot[5][32]=55;mapbot[5][33]=55;mapbot[5][34]=55;mapbot[5][35]=55;mapbot[5][36]=55;
	mapbot[6][0] =55;mapbot[6][1] = 55;mapbot[6][2]=55;mapbot[6][3]=55;mapbot[6][4]=55;mapbot[6][5]=1;mapbot[6][6]=0;mapbot[6][7]=55;mapbot[6][8]=55;mapbot[6][9]=55;mapbot[6][10]=55;mapbot[6][11]=55;mapbot[6][12]=55;mapbot[6][13]=55;mapbot[6][14]=55;mapbot[6][15]=55;mapbot[6][16]=55;mapbot[6][17]=55;mapbot[6][18]=55;mapbot[6][19]=55;mapbot[6][20]=55;mapbot[6][21]=55;mapbot[6][22]=55;mapbot[6][23]=55;mapbot[6][24]=55;mapbot[6][25]=55;mapbot[6][26]=55;mapbot[6][27]=55;mapbot[6][28]=55;mapbot[6][29]=55;mapbot[6][30]=55;mapbot[6][31]=55;mapbot[6][32]=55;mapbot[6][33]=55;mapbot[6][34]=55;mapbot[6][35]=55;mapbot[6][36]=55;
	mapbot[7][0] =1;mapbot[7][1] = 55;mapbot[7][2]=55;mapbot[7][3]=55;mapbot[7][4]=55;mapbot[7][5]=55;mapbot[7][6]=55;mapbot[7][7]=0;mapbot[7][8]=55;mapbot[7][9]=55;mapbot[7][10]=55;mapbot[7][11]=55;mapbot[7][12]=55;mapbot[7][13]=55;mapbot[7][14]=55;mapbot[7][15]=55;mapbot[7][16]=55;mapbot[7][17]=55;mapbot[7][18]=55;mapbot[7][19]=55;mapbot[7][20]=55;mapbot[7][21]=55;mapbot[7][22]=55;mapbot[7][23]=55;mapbot[7][24]=55;mapbot[7][25]=55;mapbot[7][26]=55;mapbot[7][27]=55;mapbot[7][28]=55;mapbot[7][29]=55;mapbot[7][30]=55;mapbot[7][31]=55;mapbot[7][32]=55;mapbot[7][33]=55;mapbot[7][34]=55;mapbot[7][35]=55;mapbot[7][36]=55;
	mapbot[8][0] =2;mapbot[8][1] = 55;mapbot[8][2]=55;mapbot[8][3]=55;mapbot[8][4]=55;mapbot[8][5]=55;mapbot[8][6]=55;mapbot[8][7]=55;mapbot[8][8]=0;mapbot[8][9]=2;mapbot[8][10]=55;mapbot[8][11]=55;mapbot[8][12]=55;mapbot[8][13]=55;mapbot[8][14]=55;mapbot[8][15]=55;mapbot[8][16]=55;mapbot[8][17]=55;mapbot[8][18]=55;mapbot[8][19]=55;mapbot[8][20]=55;mapbot[8][21]=55;mapbot[8][22]=55;mapbot[8][23]=55;mapbot[8][24]=55;mapbot[8][25]=55;mapbot[8][26]=55;mapbot[8][27]=55;mapbot[8][28]=55;mapbot[8][29]=55;mapbot[8][30]=3;mapbot[8][31]=55;mapbot[8][32]=55;mapbot[8][33]=55;mapbot[8][34]=55;mapbot[8][35]=55;mapbot[8][36]=55;
	mapbot[9][0] =55;mapbot[9][1] = 55;mapbot[9][2]=55;mapbot[9][3]=55;mapbot[9][4]=55;mapbot[9][5]=55;mapbot[9][6]=55;mapbot[9][7]=55;mapbot[9][8]=2;mapbot[9][9]=0;mapbot[9][10]=2;mapbot[9][11]=55;mapbot[9][12]=55;mapbot[9][13]=55;mapbot[9][14]=55;mapbot[9][15]=55;mapbot[9][16]=55;mapbot[9][17]=55;mapbot[9][18]=55;mapbot[9][19]=55;mapbot[9][20]=1;mapbot[9][21]=55;mapbot[9][22]=55;mapbot[9][23]=55;mapbot[9][24]=55;mapbot[9][25]=55;mapbot[9][26]=55;mapbot[9][27]=55;mapbot[9][28]=55;mapbot[9][29]=55;mapbot[9][30]=55;mapbot[9][31]=55;mapbot[9][32]=55;mapbot[9][33]=55;mapbot[9][34]=55;mapbot[9][35]=55;mapbot[9][36]=55;
	mapbot[10][0] = 55;mapbot[10][1] = 55;mapbot[10][2] = 55;mapbot[10][3]=55;mapbot[10][4]=55;mapbot[10][5]=1;mapbot[10][6]=55;mapbot[10][7]=55;mapbot[10][8]=55;mapbot[10][9]=2;mapbot[10][10]=0;mapbot[10][11]=1;mapbot[10][12]=55;mapbot[10][13]=55;mapbot[10][14]=55;mapbot[10][15]=55;mapbot[10][16]=55;mapbot[10][17]=55;mapbot[10][18]=55;mapbot[10][19]=55;mapbot[10][20]=55;mapbot[10][21]=55;mapbot[10][22]=3;mapbot[10][23]=55;mapbot[10][24]=55;mapbot[10][25]=55;mapbot[10][26]=55;mapbot[10][27]=55;mapbot[10][28]=55;mapbot[10][29]=55;mapbot[10][30]=55;mapbot[10][31]=55;mapbot[10][32]=55;mapbot[10][33]=55;mapbot[10][34]=55;mapbot[10][35]=55;mapbot[10][36]=55;
	mapbot[11][0] = 55;mapbot[11][1] = 55;mapbot[11][2] = 55;mapbot[11][3]=55;mapbot[11][4]=55;mapbot[11][5]=55;mapbot[11][6]=55;mapbot[11][7]=55;mapbot[11][8]=55;mapbot[11][9]=55;mapbot[11][10]=1;mapbot[11][11]=0;mapbot[11][12]=1;mapbot[11][13]=55;mapbot[11][14]=55;mapbot[11][15]=55;mapbot[11][16]=55;mapbot[11][17]=55;mapbot[11][18]=55;mapbot[11][19]=1;mapbot[11][20]=55;mapbot[11][21]=55;mapbot[11][22]=55;mapbot[11][23]=55;mapbot[11][24]=55;mapbot[11][25]=55;mapbot[11][26]=55;mapbot[11][27]=55;mapbot[11][28]=55;mapbot[11][29]=55;mapbot[11][30]=55;mapbot[11][31]=55;mapbot[11][32]=55;mapbot[11][33]=55;mapbot[11][34]=55;mapbot[11][35]=55;mapbot[11][36]=55;
	mapbot[12][0] = 55;mapbot[12][1] = 55;mapbot[12][2] = 55;mapbot[12][3]=55;mapbot[12][4]=1;mapbot[12][5]=55;mapbot[12][6]=55;mapbot[12][7]=55;mapbot[12][8]=55;mapbot[12][9]=55;mapbot[12][10]=55;mapbot[12][11]=1;mapbot[12][12]=0;mapbot[12][13]=1;mapbot[12][14]=55;mapbot[12][15]=55;mapbot[12][16]=55;mapbot[12][17]=55;mapbot[12][18]=1;mapbot[12][19]=55;mapbot[12][20]=55;mapbot[12][21]=55;mapbot[12][22]=55;mapbot[12][23]=55;mapbot[12][24]=55;mapbot[12][25]=55;mapbot[12][26]=55;mapbot[12][27]=55;mapbot[12][28]=55;mapbot[12][29]=55;mapbot[12][30]=55;mapbot[12][31]=55;mapbot[12][32]=55;mapbot[12][33]=55;mapbot[12][34]=55;mapbot[12][35]=55;mapbot[12][36]=55;
	mapbot[13][0] = 55;mapbot[13][1] = 55;mapbot[13][2] = 55;mapbot[13][3]=1;mapbot[13][4]=55;mapbot[13][5]=55;mapbot[13][6]=55;mapbot[13][7]=55;mapbot[13][8]=55;mapbot[13][9]=55;mapbot[13][10]=55;mapbot[13][11]=55;mapbot[13][12]=1;mapbot[13][13]=0;mapbot[13][14]=1;mapbot[13][15]=55;mapbot[13][16]=55;mapbot[13][17]=1;mapbot[13][18]=55;mapbot[13][19]=55;mapbot[13][20]=55;mapbot[13][21]=55;mapbot[13][22]=55;mapbot[13][23]=55;mapbot[13][24]=55;mapbot[13][25]=55;mapbot[13][26]=55;mapbot[13][27]=55;mapbot[13][28]=55;mapbot[13][29]=55;mapbot[13][30]=55;mapbot[13][31]=55;mapbot[13][32]=55;mapbot[13][33]=55;mapbot[13][34]=55;mapbot[13][35]=55;mapbot[13][36]=55;
	mapbot[14][0] = 55;mapbot[14][1] = 55;mapbot[14][2] = 1;mapbot[14][3]=55;mapbot[14][4]=55;mapbot[14][5]=55;mapbot[14][6]=55;mapbot[14][7]=55;mapbot[14][8]=55;mapbot[14][9]=55;mapbot[14][10]=55;mapbot[14][11]=55;mapbot[14][12]=55;mapbot[14][13]=1;mapbot[14][14]=0;mapbot[14][15]=1;mapbot[14][16]=1;mapbot[14][17]=55;mapbot[14][18]=55;mapbot[14][19]=55;mapbot[14][20]=55;mapbot[14][21]=55;mapbot[14][22]=55;mapbot[14][23]=55;mapbot[14][24]=55;mapbot[14][25]=55;mapbot[14][26]=55;mapbot[14][27]=55;mapbot[14][28]=55;mapbot[14][29]=55;mapbot[14][30]=55;mapbot[14][31]=55;mapbot[14][32]=55;mapbot[14][33]=55;mapbot[14][34]=55;mapbot[14][35]=55;mapbot[14][36]=55;
	mapbot[15][0] = 55;mapbot[15][1] = 6;mapbot[15][2] = 55;mapbot[15][3]=55;mapbot[15][4]=55;mapbot[15][5]=55;mapbot[15][6]=55;mapbot[15][7]=55;mapbot[15][8]=55;mapbot[15][9]=55;mapbot[15][10]=55;mapbot[15][11]=55;mapbot[15][12]=55;mapbot[15][13]=55;mapbot[15][14]=1;mapbot[15][15]=0;mapbot[15][16]=55;mapbot[15][17]=55;mapbot[15][18]=55;mapbot[15][19]=55;mapbot[15][20]=55;mapbot[15][21]=55;mapbot[15][22]=55;mapbot[15][23]=55;mapbot[15][24]=3;mapbot[15][25]=55;mapbot[15][26]=55;mapbot[15][27]=55;mapbot[15][28]=55;mapbot[15][29]=55;mapbot[15][30]=55;mapbot[15][31]=55;mapbot[15][32]=55;mapbot[15][33]=55;mapbot[15][34]=55;mapbot[15][35]=55;mapbot[15][36]=55;
	mapbot[16][0] = 55;mapbot[16][1] = 55;mapbot[16][2] = 55;mapbot[16][3]=55;mapbot[16][4]=55;mapbot[16][5]=55;mapbot[16][6]=55;mapbot[16][7]=55;mapbot[16][8]=55;mapbot[16][9]=55;mapbot[16][10]=55;mapbot[16][11]=55;mapbot[16][12]=55;mapbot[16][13]=55;mapbot[16][14]=1;mapbot[16][15]=55;mapbot[16][16]=0;mapbot[16][17]=55;mapbot[16][18]=55;mapbot[16][19]=55;mapbot[16][20]=55;mapbot[16][21]=55;mapbot[16][22]=55;mapbot[16][23]=55;mapbot[16][24]=55;mapbot[16][25]=55;mapbot[16][26]=55;mapbot[16][27]=55;mapbot[16][28]=55;mapbot[16][29]=55;mapbot[16][30]=55;mapbot[16][31]=55;mapbot[16][32]=55;mapbot[16][33]=55;mapbot[16][34]=55;mapbot[16][35]=55;mapbot[16][36]=55;
	mapbot[17][0] = 55;mapbot[17][1] = 55;mapbot[17][2] = 55;mapbot[17][3]=55;mapbot[17][4]=55;mapbot[17][5]=55;mapbot[17][6]=55;mapbot[17][7]=55;mapbot[17][8]=55;mapbot[17][9]=55;mapbot[17][10]=55;mapbot[17][11]=55;mapbot[17][12]=55;mapbot[17][13]=1;mapbot[17][14]=55;mapbot[17][15]=55;mapbot[17][16]=55;mapbot[17][17]=0;mapbot[17][18]=55;mapbot[17][19]=55;mapbot[17][20]=55;mapbot[17][21]=55;mapbot[17][22]=55;mapbot[17][23]=55;mapbot[17][24]=55;mapbot[17][25]=55;mapbot[17][26]=55;mapbot[17][27]=55;mapbot[17][28]=55;mapbot[17][29]=55;mapbot[17][30]=55;mapbot[17][31]=55;mapbot[17][32]=55;mapbot[17][33]=55;mapbot[17][34]=55;mapbot[17][35]=55;mapbot[17][36]=55;
	mapbot[18][0] = 55;mapbot[18][1] = 55;mapbot[18][2] = 55;mapbot[18][3]=55;mapbot[18][4]=55;mapbot[18][5]=55;mapbot[18][6]=55;mapbot[18][7]=55;mapbot[18][8]=55;mapbot[18][9]=55;mapbot[18][10]=55;mapbot[18][11]=55;mapbot[18][12]=1;mapbot[18][13]=55;mapbot[18][14]=55;mapbot[18][15]=55;mapbot[18][16]=55;mapbot[18][17]=55;mapbot[18][18]=0;mapbot[18][19]=55;mapbot[18][20]=55;mapbot[18][21]=55;mapbot[18][22]=55;mapbot[18][23]=55;mapbot[18][24]=55;mapbot[18][25]=55;mapbot[18][26]=55;mapbot[18][27]=55;mapbot[18][28]=55;mapbot[18][29]=55;mapbot[18][30]=55;mapbot[18][31]=55;mapbot[18][32]=55;mapbot[18][33]=55;mapbot[18][34]=55;mapbot[18][35]=55;mapbot[18][36]=55;
	mapbot[19][0] = 55;mapbot[19][1] = 55;mapbot[19][2] = 55;mapbot[19][3]=55;mapbot[19][4]=55;mapbot[19][5]=55;mapbot[19][6]=55;mapbot[19][7]=55;mapbot[19][8]=55;mapbot[19][9]=55;mapbot[19][10]=55;mapbot[19][11]=1;mapbot[19][12]=55;mapbot[19][13]=55;mapbot[19][14]=55;mapbot[19][15]=55;mapbot[19][16]=55;mapbot[19][17]=55;mapbot[19][18]=55;mapbot[19][19]=0;mapbot[19][20]=55;mapbot[19][21]=55;mapbot[19][22]=55;mapbot[19][23]=55;mapbot[19][24]=55;mapbot[19][25]=55;mapbot[19][26]=55;mapbot[19][27]=55;mapbot[19][28]=55;mapbot[19][29]=55;mapbot[19][30]=55;mapbot[19][31]=55;mapbot[19][32]=55;mapbot[19][33]=55;mapbot[19][34]=55;mapbot[19][35]=55;mapbot[19][36]=55;
	mapbot[20][0] = 55;mapbot[20][1] = 55;mapbot[20][2] = 55;mapbot[20][3]=55;mapbot[20][4]=55;mapbot[20][5]=55;mapbot[20][6]=55;mapbot[20][7]=55;mapbot[20][8]=55;mapbot[20][9]=1;mapbot[20][10]=55;mapbot[20][11]=55;mapbot[20][12]=55;mapbot[20][13]=55;mapbot[20][14]=55;mapbot[20][15]=55;mapbot[20][16]=55;mapbot[20][17]=55;mapbot[20][18]=55;mapbot[20][19]=55;mapbot[20][20]=0;mapbot[20][21]=55;mapbot[20][22]=55;mapbot[20][23]=55;mapbot[20][24]=55;mapbot[20][25]=55;mapbot[20][26]=55;mapbot[20][27]=55;mapbot[20][28]=55;mapbot[20][29]=55;mapbot[20][30]=55;mapbot[20][31]=55;mapbot[20][32]=55;mapbot[20][33]=55;mapbot[20][34]=55;mapbot[20][35]=55;mapbot[20][36]=55;
	mapbot[21][0] = 55;mapbot[21][1] = 55;mapbot[21][2] = 55;mapbot[21][3]=55;mapbot[21][4]=55;mapbot[21][5]=55;mapbot[21][6]=55;mapbot[21][7]=55;mapbot[21][8]=55;mapbot[21][9]=55;mapbot[21][10]=55;mapbot[21][11]=55;mapbot[21][12]=55;mapbot[21][13]=55;mapbot[21][14]=55;mapbot[21][15]=55;mapbot[21][16]=55;mapbot[21][17]=55;mapbot[21][18]=55;mapbot[21][19]=55;mapbot[21][20]=55;mapbot[21][21]=0;mapbot[21][22]=1;mapbot[21][23]=55;mapbot[21][24]=55;mapbot[21][25]=55;mapbot[21][26]=55;mapbot[21][27]=55;mapbot[21][28]=55;mapbot[21][29]=55;mapbot[21][30]=55;mapbot[21][31]=55;mapbot[21][32]=55;mapbot[21][33]=55;mapbot[21][34]=55;mapbot[21][35]=55;mapbot[21][36]=55;
	mapbot[22][0] = 55;mapbot[22][1] = 55;mapbot[22][2] = 55;mapbot[22][3]=55;mapbot[22][4]=55;mapbot[22][5]=55;mapbot[22][6]=55;mapbot[22][7]=55;mapbot[22][8]=55;mapbot[22][9]=55;mapbot[22][10]=3;mapbot[22][11]=55;mapbot[22][12]=55;mapbot[22][13]=55;mapbot[22][14]=55;mapbot[22][15]=55;mapbot[22][16]=55;mapbot[22][17]=55;mapbot[22][18]=55;mapbot[22][19]=55;mapbot[22][20]=55;mapbot[22][21]=1;mapbot[22][22]=0;mapbot[22][23]=2;mapbot[22][24]=55;mapbot[22][25]=55;mapbot[22][26]=55;mapbot[22][27]=55;mapbot[22][28]=1;mapbot[22][29]=55;mapbot[22][30]=55;mapbot[22][31]=55;mapbot[22][32]=55;mapbot[22][33]=55;mapbot[22][34]=55;mapbot[22][35]=55;mapbot[22][36]=55;
	mapbot[23][0] = 55;mapbot[23][1] = 55;mapbot[23][2] = 55;mapbot[23][3]=55;mapbot[23][4]=55;mapbot[23][5]=55;mapbot[23][6]=55;mapbot[23][7]=55;mapbot[23][8]=55;mapbot[23][9]=55;mapbot[23][10]=55;mapbot[23][11]=55;mapbot[23][12]=55;mapbot[23][13]=55;mapbot[23][14]=55;mapbot[23][15]=55;mapbot[23][16]=55;mapbot[23][17]=55;mapbot[23][18]=55;mapbot[23][19]=55;mapbot[23][20]=55;mapbot[23][21]=55;mapbot[23][22]=2;mapbot[23][23]=0;mapbot[23][24]=3;mapbot[23][25]=55;mapbot[23][26]=55;mapbot[23][27]=1;mapbot[23][28]=55;mapbot[23][29]=55;mapbot[23][30]=55;mapbot[23][31]=55;mapbot[23][32]=55;mapbot[23][33]=55;mapbot[23][34]=55;mapbot[23][35]=55;mapbot[23][36]=55;
	mapbot[24][0] = 55;mapbot[24][1] = 55;mapbot[24][2] = 55;mapbot[24][3]=55;mapbot[24][4]=55;mapbot[24][5]=55;mapbot[24][6]=55;mapbot[24][7]=55;mapbot[24][8]=55;mapbot[24][9]=55;mapbot[24][10]=55;mapbot[24][11]=55;mapbot[24][12]=55;mapbot[24][13]=55;mapbot[24][14]=55;mapbot[24][15]=3;mapbot[24][16]=55;mapbot[24][17]=55;mapbot[24][18]=55;mapbot[24][19]=55;mapbot[24][20]=55;mapbot[24][21]=55;mapbot[24][22]=55;mapbot[24][23]=3;mapbot[24][24]=0;mapbot[24][25]=2;mapbot[24][26]=55;mapbot[24][27]=55;mapbot[24][28]=55;mapbot[24][29]=55;mapbot[24][30]=55;mapbot[24][31]=55;mapbot[24][32]=55;mapbot[24][33]=55;mapbot[24][34]=55;mapbot[24][35]=55;mapbot[24][36]=55;
	mapbot[25][0] = 55;mapbot[25][1] = 55;mapbot[25][2] = 55;mapbot[25][3]=55;mapbot[25][4]=55;mapbot[25][5]=55;mapbot[25][6]=55;mapbot[25][7]=55;mapbot[25][8]=55;mapbot[25][9]=55;mapbot[25][10]=55;mapbot[25][11]=55;mapbot[25][12]=55;mapbot[25][13]=55;mapbot[25][14]=55;mapbot[25][15]=55;mapbot[25][16]=55;mapbot[25][17]=55;mapbot[25][18]=55;mapbot[25][19]=55;mapbot[25][20]=55;mapbot[25][21]=55;mapbot[25][22]=55;mapbot[25][23]=55;mapbot[25][24]=2;mapbot[25][25]=0;mapbot[25][26]=1;mapbot[25][27]=55;mapbot[25][28]=55;mapbot[25][29]=55;mapbot[25][30]=55;mapbot[25][31]=55;mapbot[25][32]=55;mapbot[25][33]=55;mapbot[25][34]=55;mapbot[25][35]=4;mapbot[25][36]=55;
	mapbot[26][0] = 55;mapbot[26][1] = 55;mapbot[26][2] = 55;mapbot[26][3]=55;mapbot[26][4]=55;mapbot[26][5]=55;mapbot[26][6]=55;mapbot[26][7]=55;mapbot[26][8]=55;mapbot[26][9]=55;mapbot[26][10]=55;mapbot[26][11]=55;mapbot[26][12]=55;mapbot[26][13]=55;mapbot[26][14]=55;mapbot[26][15]=55;mapbot[26][16]=55;mapbot[26][17]=55;mapbot[26][18]=55;mapbot[26][19]=55;mapbot[26][20]=55;mapbot[26][21]=55;mapbot[26][22]=55;mapbot[26][23]=55;mapbot[26][24]=55;mapbot[26][25]=1;mapbot[26][26]=0;mapbot[26][27]=55;mapbot[26][28]=55;mapbot[26][29]=55;mapbot[26][30]=55;mapbot[26][31]=55;mapbot[26][32]=55;mapbot[26][33]=55;mapbot[26][34]=55;mapbot[26][35]=55;mapbot[26][36]=55;
	mapbot[27][0] = 55;mapbot[27][1] = 55;mapbot[27][2] = 55;mapbot[27][3]=55;mapbot[27][4]=55;mapbot[27][5]=55;mapbot[27][6]=55;mapbot[27][7]=55;mapbot[27][8]=55;mapbot[27][9]=55;mapbot[27][10]=55;mapbot[27][11]=55;mapbot[27][12]=55;mapbot[27][13]=55;mapbot[27][14]=55;mapbot[27][15]=55;mapbot[27][16]=55;mapbot[27][17]=55;mapbot[27][18]=55;mapbot[27][19]=55;mapbot[27][20]=55;mapbot[27][21]=55;mapbot[27][22]=55;mapbot[27][23]=1;mapbot[27][24]=55;mapbot[27][25]=55;mapbot[27][26]=55;mapbot[27][27]=0;mapbot[27][28]=55;mapbot[27][29]=55;mapbot[27][30]=55;mapbot[27][31]=55;mapbot[27][32]=55;mapbot[27][33]=55;mapbot[27][34]=55;mapbot[27][35]=55;mapbot[27][36]=55;
	mapbot[28][0] = 55;mapbot[28][1] = 55;mapbot[28][2] = 55;mapbot[28][3]=55;mapbot[28][4]=55;mapbot[28][5]=55;mapbot[28][6]=55;mapbot[28][7]=55;mapbot[28][8]=55;mapbot[28][9]=55;mapbot[28][10]=55;mapbot[28][11]=55;mapbot[28][12]=55;mapbot[28][13]=55;mapbot[28][14]=55;mapbot[28][15]=55;mapbot[28][16]=55;mapbot[28][17]=55;mapbot[28][18]=55;mapbot[28][19]=55;mapbot[28][20]=55;mapbot[28][21]=55;mapbot[28][22]=1;mapbot[28][23]=55;mapbot[28][24]=55;mapbot[28][25]=55;mapbot[28][26]=55;mapbot[28][27]=55;mapbot[28][28]=0;mapbot[28][29]=3;mapbot[28][30]=55;mapbot[28][31]=55;mapbot[28][32]=55;mapbot[28][33]=2;mapbot[28][34]=55;mapbot[28][35]=55;mapbot[28][36]=55;
	mapbot[29][0] = 55;mapbot[29][1] = 55;mapbot[29][2] = 55;mapbot[29][3]=55;mapbot[29][4]=55;mapbot[29][5]=55;mapbot[29][6]=55;mapbot[29][7]=55;mapbot[29][8]=55;mapbot[29][9]=55;mapbot[29][10]=55;mapbot[29][11]=55;mapbot[29][12]=55;mapbot[29][13]=55;mapbot[29][14]=55;mapbot[29][15]=55;mapbot[29][16]=55;mapbot[29][17]=55;mapbot[29][18]=55;mapbot[29][19]=55;mapbot[29][20]=55;mapbot[29][21]=55;mapbot[29][22]=55;mapbot[29][23]=55;mapbot[29][24]=55;mapbot[29][25]=55;mapbot[29][26]=55;mapbot[29][27]=55;mapbot[29][28]=3;mapbot[29][29]=0;mapbot[29][30]=2;mapbot[29][31]=1;mapbot[29][32]=55;mapbot[29][33]=55;mapbot[29][34]=55;mapbot[29][35]=55;mapbot[29][36]=55;
	mapbot[30][0] = 55;mapbot[30][1] = 55;mapbot[30][2] = 55;mapbot[30][3]=55;mapbot[30][4]=55;mapbot[30][5]=55;mapbot[30][6]=55;mapbot[30][7]=55;mapbot[30][8]=3;mapbot[30][9]=55;mapbot[30][10]=55;mapbot[30][11]=55;mapbot[30][12]=55;mapbot[30][13]=55;mapbot[30][14]=55;mapbot[30][15]=55;mapbot[30][16]=55;mapbot[30][17]=55;mapbot[30][18]=55;mapbot[30][19]=55;mapbot[30][20]=55;mapbot[30][21]=55;mapbot[30][22]=55;mapbot[30][23]=55;mapbot[30][24]=55;mapbot[30][25]=55;mapbot[30][26]=55;mapbot[30][27]=55;mapbot[30][28]=55;mapbot[30][29]=2;mapbot[30][30]=0;mapbot[30][31]=55;mapbot[30][32]=55;mapbot[30][33]=55;mapbot[30][34]=55;mapbot[30][35]=55;mapbot[30][36]=4;
	mapbot[31][0] = 55;mapbot[31][1] = 55;mapbot[31][2] = 55;mapbot[31][3]=55;mapbot[31][4]=55;mapbot[31][5]=55;mapbot[31][6]=55;mapbot[31][7]=55;mapbot[31][8]=55;mapbot[31][9]=55;mapbot[31][10]=55;mapbot[31][11]=55;mapbot[31][12]=55;mapbot[31][13]=55;mapbot[31][14]=55;mapbot[31][15]=55;mapbot[31][16]=55;mapbot[31][17]=55;mapbot[31][18]=55;mapbot[31][19]=55;mapbot[31][20]=55;mapbot[31][21]=55;mapbot[31][22]=55;mapbot[31][23]=55;mapbot[31][24]=55;mapbot[31][25]=55;mapbot[31][26]=55;mapbot[31][27]=55;mapbot[31][28]=55;mapbot[31][29]=1;mapbot[31][30]=55;mapbot[31][31]=0;mapbot[31][32]=55;mapbot[31][33]=55;mapbot[31][34]=55;mapbot[31][35]=55;mapbot[31][36]=55;
	mapbot[32][0] = 55;mapbot[32][1] = 55;mapbot[32][2] = 55;mapbot[32][3]=55;mapbot[32][4]=55;mapbot[32][5]=55;mapbot[32][6]=55;mapbot[32][7]=55;mapbot[32][8]=55;mapbot[32][9]=55;mapbot[32][10]=55;mapbot[32][11]=55;mapbot[32][12]=55;mapbot[32][13]=55;mapbot[32][14]=55;mapbot[32][15]=55;mapbot[32][16]=55;mapbot[32][17]=55;mapbot[32][18]=55;mapbot[32][19]=55;mapbot[32][20]=55;mapbot[32][21]=55;mapbot[32][22]=55;mapbot[32][23]=55;mapbot[32][24]=55;mapbot[32][25]=55;mapbot[32][26]=55;mapbot[32][27]=55;mapbot[32][28]=55;mapbot[32][29]=55;mapbot[32][30]=55;mapbot[32][31]=55;mapbot[32][32]=0;mapbot[32][33]=1;mapbot[32][34]=55;mapbot[32][35]=55;mapbot[32][36]=55;
	mapbot[33][0] = 55;mapbot[33][1] = 55;mapbot[33][2] = 55;mapbot[33][3]=55;mapbot[33][4]=55;mapbot[33][5]=55;mapbot[33][6]=55;mapbot[33][7]=55;mapbot[33][8]=55;mapbot[33][9]=55;mapbot[33][10]=55;mapbot[33][11]=55;mapbot[33][12]=55;mapbot[33][13]=55;mapbot[33][14]=55;mapbot[33][15]=55;mapbot[33][16]=55;mapbot[33][17]=55;mapbot[33][18]=55;mapbot[33][19]=55;mapbot[33][20]=55;mapbot[33][21]=55;mapbot[33][22]=55;mapbot[33][23]=55;mapbot[33][24]=55;mapbot[33][25]=55;mapbot[33][26]=55;mapbot[33][27]=55;mapbot[33][28]=2;mapbot[33][29]=55;mapbot[33][30]=55;mapbot[33][31]=55;mapbot[33][32]=1;mapbot[33][33]=0;mapbot[33][34]=55;mapbot[33][35]=55;mapbot[33][36]=1;
	mapbot[34][0] = 55;mapbot[34][1] = 55;mapbot[34][2] = 55;mapbot[34][3]=55;mapbot[34][4]=55;mapbot[34][5]=55;mapbot[34][6]=55;mapbot[34][7]=55;mapbot[34][8]=55;mapbot[34][9]=55;mapbot[34][10]=55;mapbot[34][11]=55;mapbot[34][12]=55;mapbot[34][13]=55;mapbot[34][14]=55;mapbot[34][15]=55;mapbot[34][16]=55;mapbot[34][17]=55;mapbot[34][18]=55;mapbot[34][19]=55;mapbot[34][20]=55;mapbot[34][21]=55;mapbot[34][22]=55;mapbot[34][23]=55;mapbot[34][24]=55;mapbot[34][25]=55;mapbot[34][26]=55;mapbot[34][27]=55;mapbot[34][28]=55;mapbot[34][29]=55;mapbot[34][30]=55;mapbot[34][31]=55;mapbot[34][32]=55;mapbot[34][33]=55;mapbot[34][34]=0;mapbot[34][35]=1;mapbot[34][36]=55;
	mapbot[35][0] = 55;mapbot[35][1] = 55;mapbot[35][2] = 55;mapbot[35][3]=55;mapbot[35][4]=55;mapbot[35][5]=55;mapbot[35][6]=55;mapbot[35][7]=55;mapbot[35][8]=55;mapbot[35][9]=55;mapbot[35][10]=55;mapbot[35][11]=55;mapbot[35][12]=55;mapbot[35][13]=55;mapbot[35][14]=55;mapbot[35][15]=55;mapbot[35][16]=55;mapbot[35][17]=55;mapbot[35][18]=55;mapbot[35][19]=55;mapbot[35][20]=55;mapbot[35][21]=55;mapbot[35][22]=55;mapbot[35][23]=55;mapbot[35][24]=55;mapbot[35][25]=4;mapbot[35][26]=55;mapbot[35][27]=55;mapbot[35][28]=55;mapbot[35][29]=55;mapbot[35][30]=55;mapbot[35][31]=55;mapbot[35][32]=55;mapbot[35][33]=55;mapbot[35][34]=1;mapbot[35][35]=0;mapbot[35][36]=2;
	mapbot[36][0] = 55;mapbot[36][1] = 55;mapbot[36][2] = 55;mapbot[36][3]=55;mapbot[36][4]=55;mapbot[36][5]=55;mapbot[36][6]=55;mapbot[36][7]=55;mapbot[36][8]=55;mapbot[36][9]=55;mapbot[36][10]=55;mapbot[36][11]=55;mapbot[36][12]=55;mapbot[36][13]=55;mapbot[36][14]=55;mapbot[36][15]=55;mapbot[36][16]=55;mapbot[36][17]=55;mapbot[36][18]=55;mapbot[36][19]=55;mapbot[36][20]=55;mapbot[36][21]=55;mapbot[36][22]=55;mapbot[36][23]=55;mapbot[36][24]=55;mapbot[36][25]=55;mapbot[36][26]=55;mapbot[36][27]=55;mapbot[36][28]=55;mapbot[36][29]=55;mapbot[36][30]=4;mapbot[36][31]=55;mapbot[36][32]=55;mapbot[36][33]=1;mapbot[36][34]=55;mapbot[36][35]=2;mapbot[36][36]=0;
	
	noddir[0][0] = 55;  noddir[0][1] = 3;   noddir[0][2] = 55;  noddir[0][3] = 55;  noddir[0][4]=55;noddir[0][5]=55;noddir[0][6]=55;noddir[0][7]=0;noddir[0][8]=1;noddir[0][9]=55;noddir[0][10]=55;noddir[0][11]=55;noddir[0][12]=55;noddir[0][13]=55;noddir[0][14]=55;noddir[0][15]=55;noddir[0][16]=55;noddir[0][17]=55;noddir[0][18]=55;noddir[0][19]=55;noddir[0][20]=55;noddir[0][21]=55;noddir[0][22]=55;noddir[0][23]=55;noddir[0][24]=55;noddir[0][25]=55;noddir[0][26]=55;noddir[0][27]=55;noddir[0][28]=55;noddir[0][29]=55;noddir[0][30]=55;noddir[0][31]=55;noddir[0][32]=55;noddir[0][33]=55;noddir[0][34]=55;noddir[0][35]=55;noddir[0][36]=55;
	noddir[1][0] = 2;   noddir[1][1] = 55;  noddir[1][2] = 55;  noddir[1][3] = 55;  noddir[1][4]=55;noddir[1][5]=1;noddir[1][6]=55;noddir[1][7]=55;noddir[1][8]=55;noddir[1][9]=55;noddir[1][10]=55;noddir[1][11]=55;noddir[1][12]=55;noddir[1][13]=55;noddir[1][14]=55;noddir[1][15]=0;noddir[1][16]=55;noddir[1][17]=55;noddir[1][18]=55;noddir[1][19]=55;noddir[1][20]=55;noddir[1][21]=55;noddir[1][22]=55;noddir[1][23]=55;noddir[1][24]=55;noddir[1][25]=55;noddir[1][26]=55;noddir[1][27]=55;noddir[1][28]=55;noddir[1][29]=55;noddir[1][30]=55;noddir[1][31]=55;noddir[1][32]=55;noddir[1][33]=55;noddir[1][34]=55;noddir[1][35]=55;noddir[1][36]=55;
	noddir[2][0] = 55;  noddir[2][1] = 55;  noddir[2][2] = 55;  noddir[2][3] = 55;  noddir[2][4]=55;noddir[2][5]=55;noddir[2][6]=55;noddir[2][7]=55;noddir[2][8]=55;noddir[2][9]=55;noddir[2][10]=55;noddir[2][11]=55;noddir[2][12]=55;noddir[2][13]=55;noddir[2][14]=1;noddir[2][15]=55;noddir[2][16]=55;noddir[2][17]=55;noddir[2][18]=55;noddir[2][19]=55;noddir[2][20]=55;noddir[2][21]=55;noddir[2][22]=55;noddir[2][23]=55;noddir[2][24]=55;noddir[2][25]=55;noddir[2][26]=55;noddir[2][27]=55;noddir[2][28]=55;noddir[2][29]=55;noddir[2][30]=55;noddir[2][31]=55;noddir[2][32]=55;noddir[2][33]=55;noddir[2][34]=55;noddir[2][35]=55;noddir[2][36]=55;
	noddir[3][0] = 55;  noddir[3][1] = 55;  noddir[3][2] = 55;  noddir[3][3] = 55;  noddir[3][4]=55;noddir[3][5]=55;noddir[3][6]=55;noddir[3][7]=55;noddir[3][8]=55;noddir[3][9]=55;noddir[3][10]=55;noddir[3][11]=55;noddir[3][12]=55;noddir[3][13]=1;noddir[3][14]=55;noddir[3][15]=55;noddir[3][16]=55;noddir[3][17]=55;noddir[3][18]=55;noddir[3][19]=55;noddir[3][20]=55;noddir[3][21]=55;noddir[3][22]=55;noddir[3][23]=55;noddir[3][24]=55;noddir[3][25]=55;noddir[3][26]=55;noddir[3][27]=55;noddir[3][28]=55;noddir[3][29]=55;noddir[3][30]=55;noddir[3][31]=55;noddir[3][32]=55;noddir[3][33]=55;noddir[3][34]=55;noddir[3][35]=55;noddir[3][36]=55;
	noddir[4][0] = 55;  noddir[4][1] = 55;  noddir[4][2] = 55;  noddir[4][3] = 55;  noddir[4][4]=55;noddir[4][5]=55;noddir[4][6]=55;noddir[4][7]=55;noddir[4][8]=55;noddir[4][9]=55;noddir[4][10]=55;noddir[4][11]=55;noddir[4][12]=1;noddir[4][13]=55;noddir[4][14]=55;noddir[4][15]=55;noddir[4][16]=55;noddir[4][17]=55;noddir[4][18]=55;noddir[4][19]=55;noddir[4][20]=55;noddir[4][21]=55;noddir[4][22]=55;noddir[4][23]=55;noddir[4][24]=55;noddir[4][25]=55;noddir[4][26]=55;noddir[4][27]=55;noddir[4][28]=55;noddir[4][29]=55;noddir[4][30]=55;noddir[4][31]=55;noddir[4][32]=55;noddir[4][33]=55;noddir[4][34]=55;noddir[4][35]=55;noddir[4][36]=55;
	noddir[5][0] = 55;  noddir[5][1] = 3;   noddir[5][2] = 55;  noddir[5][3] = 55;  noddir[5][4]=55;noddir[5][5]=55;noddir[5][6]=2;noddir[5][7]=55;noddir[5][8]=55;noddir[5][9]=55;noddir[5][10]=1;noddir[5][11]=55;noddir[5][12]=55;noddir[5][13]=55;noddir[5][14]=55;noddir[5][15]=55;noddir[5][16]=55;noddir[5][17]=55;noddir[5][18]=55;noddir[5][19]=55;noddir[5][20]=55;noddir[5][21]=55;noddir[5][22]=55;noddir[5][23]=55;noddir[5][24]=55;noddir[5][25]=55;noddir[5][26]=55;noddir[5][27]=55;noddir[5][28]=55;noddir[5][29]=55;noddir[5][30]=55;noddir[5][31]=55;noddir[5][32]=55;noddir[5][33]=55;noddir[5][34]=55;noddir[5][35]=55;noddir[5][36]=55;
	noddir[6][0] = 55;  noddir[6][1] = 55;  noddir[6][2] = 55;  noddir[6][3] = 55;  noddir[6][4]=55;noddir[6][5]=0;noddir[6][6]=55;noddir[6][7]=55;noddir[6][8]=55;noddir[6][9]=55;noddir[6][10]=55;noddir[6][11]=55;noddir[6][12]=55;noddir[6][13]=55;noddir[6][14]=55;noddir[6][15]=55;noddir[6][16]=55;noddir[6][17]=55;noddir[6][18]=55;noddir[6][19]=55;noddir[6][20]=55;noddir[6][21]=55;noddir[6][22]=55;noddir[6][23]=55;noddir[6][24]=55;noddir[6][25]=55;noddir[6][26]=55;noddir[6][27]=55;noddir[6][28]=55;noddir[6][29]=55;noddir[6][30]=55;noddir[6][31]=55;noddir[6][32]=55;noddir[6][33]=55;noddir[6][34]=55;noddir[6][35]=55;noddir[6][36]=55;
	noddir[7][0] = 2;   noddir[7][1] = 55;  noddir[7][2] = 55;  noddir[7][3] = 55;  noddir[7][4]=55;noddir[7][5]=55;noddir[7][6]=55;noddir[7][7]=55;noddir[7][8]=55;noddir[7][9]=55;noddir[7][10]=55;noddir[7][11]=55;noddir[7][12]=55;noddir[7][13]=55;noddir[7][14]=55;noddir[7][15]=55;noddir[7][16]=55;noddir[7][17]=55;noddir[7][18]=55;noddir[7][19]=55;noddir[7][20]=55;noddir[7][21]=55;noddir[7][22]=55;noddir[7][23]=55;noddir[7][24]=55;noddir[7][25]=55;noddir[7][26]=55;noddir[7][27]=55;noddir[7][28]=55;noddir[7][29]=55;noddir[7][30]=55;noddir[7][31]=55;noddir[7][32]=55;noddir[7][33]=55;noddir[7][34]=55;noddir[7][35]=55;noddir[7][36]=55;
	noddir[8][0] = 3;   noddir[8][1] = 55;  noddir[8][2] = 55;  noddir[8][3] = 55;  noddir[8][4]=55;noddir[8][5]=55;noddir[8][6]=55;noddir[8][7]=55;noddir[8][8]=55;noddir[8][9]=0;noddir[8][10]=55;noddir[8][11]=55;noddir[8][12]=55;noddir[8][13]=55;noddir[8][14]=55;noddir[8][15]=55;noddir[8][16]=55;noddir[8][17]=55;noddir[8][18]=55;noddir[8][19]=55;noddir[8][20]=55;noddir[8][21]=55;noddir[8][22]=55;noddir[8][23]=55;noddir[8][24]=55;noddir[8][25]=55;noddir[8][26]=55;noddir[8][27]=55;noddir[8][28]=55;noddir[8][29]=55;noddir[8][30]=1;noddir[8][31]=55;noddir[8][32]=55;noddir[8][33]=55;noddir[8][34]=55;noddir[8][35]=55;noddir[8][36]=55;
	noddir[9][0] = 55;  noddir[9][1] = 55;  noddir[9][2] = 55;  noddir[9][3] = 55;  noddir[9][4]=55;noddir[9][5]=55;noddir[9][6]=55;noddir[9][7]=55;noddir[9][8]=2;noddir[9][9]=55;noddir[9][10]=0;noddir[9][11]=55;noddir[9][12]=55;noddir[9][13]=55;noddir[9][14]=55;noddir[9][15]=55;noddir[9][16]=55;noddir[9][17]=55;noddir[9][18]=55;noddir[9][19]=55;noddir[9][20]=1;noddir[9][21]=55;noddir[9][22]=55;noddir[9][23]=55;noddir[9][24]=55;noddir[9][25]=55;noddir[9][26]=55;noddir[9][27]=55;noddir[9][28]=55;noddir[9][29]=55;noddir[9][30]=55;noddir[9][31]=55;noddir[9][32]=55;noddir[9][33]=55;noddir[9][34]=55;noddir[9][35]=55;noddir[9][36]=55;
	noddir[10][0] = 55; noddir[10][1] = 55; noddir[10][2] = 55; noddir[10][3] = 55; noddir[10][4]=55;noddir[10][5]=3;noddir[10][6]=55;noddir[10][7]=55;noddir[10][8]=55;noddir[10][9]=2;noddir[10][10]=55;noddir[10][11]=0;noddir[10][12]=55;noddir[10][13]=55;noddir[10][14]=55;noddir[10][15]=55;noddir[10][16]=55;noddir[10][17]=55;noddir[10][18]=55;noddir[10][19]=55;noddir[10][20]=55;noddir[10][21]=55;noddir[10][22]=1;noddir[10][23]=55;noddir[10][24]=55;noddir[10][25]=55;noddir[10][26]=55;noddir[10][27]=55;noddir[10][28]=55;noddir[10][29]=55;noddir[10][30]=55;noddir[10][31]=55;noddir[10][32]=55;noddir[10][33]=55;noddir[10][34]=55;noddir[10][35]=55;noddir[10][36]=55;
	noddir[11][0] = 55; noddir[11][1] = 55; noddir[11][2] = 55; noddir[11][3] = 55; noddir[11][4]=55;noddir[11][5]=55;noddir[11][6]=55;noddir[11][7]=55;noddir[11][8]=55;noddir[11][9]=55;noddir[11][10]=2;noddir[11][11]=55;noddir[11][12]=0;noddir[11][13]=55;noddir[11][14]=55;noddir[11][15]=55;noddir[11][16]=55;noddir[11][17]=55;noddir[11][18]=55;noddir[11][19]=1;noddir[11][20]=55;noddir[11][21]=55;noddir[11][22]=55;noddir[11][23]=55;noddir[11][24]=55;noddir[11][25]=55;noddir[11][26]=55;noddir[11][27]=55;noddir[11][28]=55;noddir[11][29]=55;noddir[11][30]=55;noddir[11][31]=55;noddir[11][32]=55;noddir[11][33]=55;noddir[11][34]=55;noddir[11][35]=55;noddir[11][36]=55;
	noddir[12][0] = 55; noddir[12][1] = 55; noddir[12][2] = 55; noddir[12][3] = 55; noddir[12][4]=3;noddir[12][5]=55;noddir[12][6]=55;noddir[12][7]=55;noddir[12][8]=55;noddir[12][9]=55;noddir[12][10]=55;noddir[12][11]=2;noddir[12][12]=55;noddir[12][13]=0;noddir[12][14]=55;noddir[12][15]=55;noddir[12][16]=55;noddir[12][17]=55;noddir[12][18]=1;noddir[12][19]=55;noddir[12][20]=55;noddir[12][21]=55;noddir[12][22]=55;noddir[12][23]=55;noddir[12][24]=55;noddir[12][25]=55;noddir[12][26]=55;noddir[12][27]=55;noddir[12][28]=55;noddir[12][29]=55;noddir[12][30]=55;noddir[12][31]=55;noddir[12][32]=55;noddir[12][33]=55;noddir[12][34]=55;noddir[12][35]=55;noddir[12][36]=55;
	noddir[13][0] = 55; noddir[13][1] = 55; noddir[13][2] = 55; noddir[13][3] = 3;  noddir[13][4]=55;noddir[13][5]=55;noddir[13][6]=55;noddir[13][7]=55;noddir[13][8]=55;noddir[13][9]=55;noddir[13][10]=55;noddir[13][11]=55;noddir[13][12]=2;noddir[13][13]=55;noddir[13][14]=0;noddir[13][15]=55;noddir[13][16]=55;noddir[13][17]=1;noddir[13][18]=55;noddir[13][19]=55;noddir[13][20]=55;noddir[13][21]=55;noddir[13][22]=55;noddir[13][23]=55;noddir[13][24]=55;noddir[13][25]=55;noddir[13][26]=55;noddir[13][27]=55;noddir[13][28]=55;noddir[13][29]=55;noddir[13][30]=55;noddir[13][31]=55;noddir[13][32]=55;noddir[13][33]=55;noddir[13][34]=55;noddir[13][35]=55;noddir[13][36]=55;
	noddir[14][0] = 55; noddir[14][1] = 55; noddir[14][2] = 3;  noddir[14][3] = 55; noddir[14][4]=55;noddir[14][5]=55;noddir[14][6]=55;noddir[14][7]=55;noddir[14][8]=55;noddir[14][9]=55;noddir[14][10]=55;noddir[14][11]=55;noddir[14][12]=55;noddir[14][13]=2;noddir[14][14]=55;noddir[14][15]=0;noddir[14][16]=1;noddir[14][17]=55;noddir[14][18]=55;noddir[14][19]=55;noddir[14][20]=55;noddir[14][21]=55;noddir[14][22]=55;noddir[14][23]=55;noddir[14][24]=55;noddir[14][25]=55;noddir[14][26]=55;noddir[14][27]=55;noddir[14][28]=55;noddir[14][29]=55;noddir[14][30]=55;noddir[14][31]=55;noddir[14][32]=55;noddir[14][33]=55;noddir[14][34]=55;noddir[14][35]=55;noddir[14][36]=55;
	noddir[15][0] = 55; noddir[15][1] = 3;  noddir[15][2] = 55; noddir[15][3] = 55; noddir[15][4]=55;noddir[15][5]=55;noddir[15][6]=55;noddir[15][7]=55;noddir[15][8]=55;noddir[15][9]=55;noddir[15][10]=55;noddir[15][11]=55;noddir[15][12]=55;noddir[15][13]=55;noddir[15][14]=2;noddir[15][15]=55;noddir[15][16]=55;noddir[15][17]=55;noddir[15][18]=55;noddir[15][19]=55;noddir[15][20]=55;noddir[15][21]=55;noddir[15][22]=55;noddir[15][23]=55;noddir[15][24]=1;noddir[15][25]=55;noddir[15][26]=55;noddir[15][27]=55;noddir[15][28]=55;noddir[15][29]=55;noddir[15][30]=55;noddir[15][31]=55;noddir[15][32]=55;noddir[15][33]=55;noddir[15][34]=55;noddir[15][35]=55;noddir[15][36]=55;
	noddir[16][0] = 55; noddir[16][1] = 55; noddir[16][2] = 55; noddir[16][3] = 55; noddir[16][4]=55;noddir[16][5]=55;noddir[16][6]=55;noddir[16][7]=55;noddir[16][8]=55;noddir[16][9]=55;noddir[16][10]=55;noddir[16][11]=55;noddir[16][12]=55;noddir[16][13]=55;noddir[16][14]=3;noddir[16][15]=55;noddir[16][16]=55;noddir[16][17]=55;noddir[16][18]=55;noddir[16][19]=55;noddir[16][20]=55;noddir[16][21]=55;noddir[16][22]=55;noddir[16][23]=55;noddir[16][24]=55;noddir[16][25]=55;noddir[16][26]=55;noddir[16][27]=55;noddir[16][28]=55;noddir[16][29]=55;noddir[16][30]=55;noddir[16][31]=55;noddir[16][32]=55;noddir[16][33]=55;noddir[16][34]=55;noddir[16][35]=55;noddir[16][36]=55;
	noddir[17][0] = 55; noddir[17][1] = 55; noddir[17][2] = 55; noddir[17][3] = 55; noddir[17][4]=55;noddir[17][5]=55;noddir[17][6]=55;noddir[17][7]=55;noddir[17][8]=55;noddir[17][9]=55;noddir[17][10]=55;noddir[17][11]=55;noddir[17][12]=55;noddir[17][13]=3;noddir[17][14]=55;noddir[17][15]=55;noddir[17][16]=55;noddir[17][17]=55;noddir[17][18]=55;noddir[17][19]=55;noddir[17][20]=55;noddir[17][21]=55;noddir[17][22]=55;noddir[17][23]=55;noddir[17][24]=55;noddir[17][25]=55;noddir[17][26]=55;noddir[17][27]=55;noddir[17][28]=55;noddir[17][29]=55;noddir[17][30]=55;noddir[17][31]=55;noddir[17][32]=55;noddir[17][33]=55;noddir[17][34]=55;noddir[17][35]=55;noddir[17][36]=55;
	noddir[18][0] = 55; noddir[18][1] = 55; noddir[18][2] = 55; noddir[18][3] = 55; noddir[18][4]=55;noddir[18][5]=55;noddir[18][6]=55;noddir[18][7]=55;noddir[18][8]=55;noddir[18][9]=55;noddir[18][10]=55;noddir[18][11]=55;noddir[18][12]=3;noddir[18][13]=55;noddir[18][14]=55;noddir[18][15]=55;noddir[18][16]=55;noddir[18][17]=55;noddir[18][18]=55;noddir[18][19]=55;noddir[18][20]=55;noddir[18][21]=55;noddir[18][22]=55;noddir[18][23]=55;noddir[18][24]=55;noddir[18][25]=55;noddir[18][26]=55;noddir[18][27]=55;noddir[18][28]=55;noddir[18][29]=55;noddir[18][30]=55;noddir[18][31]=55;noddir[18][32]=55;noddir[18][33]=55;noddir[18][34]=55;noddir[18][35]=55;noddir[18][36]=55;
	noddir[19][0] = 55; noddir[19][1] = 55; noddir[19][2] = 55; noddir[19][3] = 55; noddir[19][4]=55;noddir[19][5]=55;noddir[19][6]=55;noddir[19][7]=55;noddir[19][8]=55;noddir[19][9]=55;noddir[19][10]=55;noddir[19][11]=3;noddir[19][12]=55;noddir[19][13]=55;noddir[19][14]=55;noddir[19][15]=55;noddir[19][16]=55;noddir[19][17]=55;noddir[19][18]=55;noddir[19][19]=55;noddir[19][20]=55;noddir[19][21]=55;noddir[19][22]=55;noddir[19][23]=55;noddir[19][24]=55;noddir[19][25]=55;noddir[19][26]=55;noddir[19][27]=55;noddir[19][28]=55;noddir[19][29]=55;noddir[19][30]=55;noddir[19][31]=55;noddir[19][32]=55;noddir[19][33]=55;noddir[19][34]=55;noddir[19][35]=55;noddir[19][36]=55;
	noddir[20][0] = 55; noddir[20][1] = 55; noddir[20][2] = 55; noddir[20][3] = 55; noddir[20][4]=55;noddir[20][5]=55;noddir[20][6]=55;noddir[20][7]=55;noddir[20][8]=55;noddir[20][9]=3;noddir[20][10]=55;noddir[20][11]=55;noddir[20][12]=55;noddir[20][13]=55;noddir[20][14]=55;noddir[20][15]=55;noddir[20][16]=55;noddir[20][17]=55;noddir[20][18]=55;noddir[20][19]=55;noddir[20][20]=55;noddir[20][21]=55;noddir[20][22]=55;noddir[20][23]=55;noddir[20][24]=55;noddir[20][25]=55;noddir[20][26]=55;noddir[20][27]=55;noddir[20][28]=55;noddir[20][29]=55;noddir[20][30]=55;noddir[20][31]=55;noddir[20][32]=55;noddir[20][33]=55;noddir[20][34]=55;noddir[20][35]=55;noddir[20][36]=55;
	noddir[21][0] = 55; noddir[21][1] = 55; noddir[21][2] = 55; noddir[21][3] = 55; noddir[21][4]=55;noddir[21][5]=55;noddir[21][6]=55;noddir[21][7]=55;noddir[21][8]=55;noddir[21][9]=55;noddir[21][10]=55;noddir[21][11]=55;noddir[21][12]=55;noddir[21][13]=55;noddir[21][14]=55;noddir[21][15]=55;noddir[21][16]=55;noddir[21][17]=55;noddir[21][18]=55;noddir[21][19]=55;noddir[21][20]=55;noddir[21][21]=55;noddir[21][22]=0;noddir[21][23]=55;noddir[21][24]=55;noddir[21][25]=55;noddir[21][26]=55;noddir[21][27]=55;noddir[21][28]=55;noddir[21][29]=55;noddir[21][30]=55;noddir[21][31]=55;noddir[21][32]=55;noddir[21][33]=55;noddir[21][34]=55;noddir[21][35]=55;noddir[21][36]=55;
	noddir[22][0] = 55; noddir[22][1] = 55; noddir[22][2] = 55; noddir[22][3] = 55; noddir[22][4] = 55;noddir[22][5] = 55;noddir[22][6] = 55;noddir[22][7] = 55;noddir[22][8] = 55;noddir[22][9] = 55;noddir[22][10] = 3;noddir[22][11] = 55;noddir[22][12] = 55;noddir[22][13] = 55;noddir[22][14] = 55;noddir[22][15] = 55;noddir[22][16] = 55;noddir[22][17] = 55;noddir[22][18] = 55;noddir[22][19] = 55;noddir[22][20] = 55;noddir[22][21] = 2;noddir[22][22] = 55;noddir[22][23] = 0;noddir[22][24] = 55;noddir[22][25] = 55;noddir[22][26] = 55;noddir[22][27] = 55;noddir[22][28] = 1;noddir[22][29] = 55;noddir[22][30] = 55;noddir[22][31] = 55;noddir[22][32] = 55;noddir[22][33] = 55;noddir[22][34] = 55;noddir[22][35] = 55;noddir[22][36] = 55;
	noddir[23][0]  =  55; noddir[23][1]  =  55; noddir[23][2]  =  55; noddir[23][3] = 55; noddir[23][4] = 55;noddir[23][5] = 55;noddir[23][6] = 55;noddir[23][7] = 55;noddir[23][8] = 55;noddir[23][9] = 55;noddir[23][10] = 55;noddir[23][11] = 55;noddir[23][12] = 55;noddir[23][13] = 55;noddir[23][14] = 55;noddir[23][15] = 55;noddir[23][16] = 55;noddir[23][17] = 55;noddir[23][18] = 55;noddir[23][19] = 55;noddir[23][20] = 55;noddir[23][21] = 55;noddir[23][22] = 2;noddir[23][23] = 55;noddir[23][24] = 0;noddir[23][25] = 55;noddir[23][26] = 55;noddir[23][27] = 1;noddir[23][28] = 55;noddir[23][29] = 55;noddir[23][30] = 55;noddir[23][31] = 55;noddir[23][32] = 55;noddir[23][33] = 55;noddir[23][34] = 55;noddir[23][35] = 55;noddir[23][36] = 55;
	noddir[24][0]  =  55; noddir[24][1]  =  55; noddir[24][2]  =  55; noddir[24][3] = 55; noddir[24][4] = 55;noddir[24][5] = 55;noddir[24][6] = 55;noddir[24][7] = 55;noddir[24][8] = 55;noddir[24][9] = 55;noddir[24][10] = 55;noddir[24][11] = 55;noddir[24][12] = 55;noddir[24][13] = 55;noddir[24][14] = 55;noddir[24][15] = 3;noddir[24][16] = 55;noddir[24][17] = 55;noddir[24][18] = 55;noddir[24][19] = 55;noddir[24][20] = 55;noddir[24][21] = 55;noddir[24][22] = 55;noddir[24][23] = 2;noddir[24][24] = 55;noddir[24][25] = 1;noddir[24][26] = 55;noddir[24][27] = 55;noddir[24][28] = 55;noddir[24][29] = 55;noddir[24][30] = 55;noddir[24][31] = 55;noddir[24][32] = 55;noddir[24][33] = 55;noddir[24][34] = 55;noddir[24][35] = 55;noddir[24][36] = 55;
	noddir[25][0]  =  55; noddir[25][1]  =  55; noddir[25][2]  =  55; noddir[25][3] = 55; noddir[25][4] = 55;noddir[25][5] = 55;noddir[25][6] = 55;noddir[25][7] = 55;noddir[25][8] = 55;noddir[25][9] = 55;noddir[25][10] = 55;noddir[25][11] = 55;noddir[25][12] = 55;noddir[25][13] = 55;noddir[25][14] = 55;noddir[25][15] = 55;noddir[25][16] = 55;noddir[25][17] = 55;noddir[25][18] = 55;noddir[25][19] = 55;noddir[25][20] = 55;noddir[25][21] = 55;noddir[25][22] = 55;noddir[25][23] = 55;noddir[25][24] = 3;noddir[25][25] = 55;noddir[25][26] = 2;noddir[25][27] = 55;noddir[25][28] = 55;noddir[25][29] = 55;noddir[25][30] = 55;noddir[25][31] = 55;noddir[25][32] = 55;noddir[25][33] = 55;noddir[25][34] = 55;noddir[25][35] = 1;noddir[25][36] = 55;
	noddir[26][0]  =  55; noddir[26][1]  =  55; noddir[26][2]  =  55; noddir[26][3] = 55; noddir[26][4] = 55;noddir[26][5] = 55;noddir[26][6] = 55;noddir[26][7] = 55;noddir[26][8] = 55;noddir[26][9] = 55;noddir[26][10] = 55;noddir[26][11] = 55;noddir[26][12] = 55;noddir[26][13] = 55;noddir[26][14] = 55;noddir[26][15] = 55;noddir[26][16] = 55;noddir[26][17] = 55;noddir[26][18] = 55;noddir[26][19] = 55;noddir[26][20] = 55;noddir[26][21] = 55;noddir[26][22] = 55;noddir[26][23] = 55;noddir[26][24] = 55;noddir[26][25] = 0;noddir[26][26] = 55;noddir[26][27] = 55;noddir[26][28] = 55;noddir[26][29] = 55;noddir[26][30] = 55;noddir[26][31] = 55;noddir[26][32] = 55;noddir[26][33] = 55;noddir[26][34] = 55;noddir[26][35] = 55;noddir[26][36] = 55;
	noddir[27][0]  =  55; noddir[27][1]  =  55; noddir[27][2]  =  55; noddir[27][3] = 55; noddir[27][4] = 55;noddir[27][5] = 55;noddir[27][6] = 55;noddir[27][7] = 55;noddir[27][8] = 55;noddir[27][9] = 55;noddir[27][10] = 55;noddir[27][11] = 55;noddir[27][12] = 55;noddir[27][13] = 55;noddir[27][14] = 55;noddir[27][15] = 55;noddir[27][16] = 55;noddir[27][17] = 55;noddir[27][18] = 55;noddir[27][19] = 55;noddir[27][20] = 55;noddir[27][21] = 55;noddir[27][22] = 55;noddir[27][23] = 3;noddir[27][24] = 55;noddir[27][25] = 55;noddir[27][26] = 55;noddir[27][27] = 55;noddir[27][28] = 55;noddir[27][29] = 55;noddir[27][30] = 55;noddir[27][31] = 55;noddir[27][32] = 55;noddir[27][33] = 55;noddir[27][34] = 55;noddir[27][35] = 55;noddir[27][36] = 55;
	noddir[28][0]  =  55; noddir[28][1]  =  55; noddir[28][2]  =  55; noddir[28][3] = 55; noddir[28][4] = 55;noddir[28][5] = 55;noddir[28][6] = 55;noddir[28][7] = 55;noddir[28][8] = 55;noddir[28][9] = 55;noddir[28][10] = 55;noddir[28][11] = 55;noddir[28][12] = 55;noddir[28][13] = 55;noddir[28][14] = 55;noddir[28][15] = 55;noddir[28][16] = 55;noddir[28][17] = 55;noddir[28][18] = 55;noddir[28][19] = 55;noddir[28][20] = 55;noddir[28][21] = 55;noddir[28][22] = 3;noddir[28][23] = 55;noddir[28][24] = 55;noddir[28][25] = 55;noddir[28][26] = 55;noddir[28][27] = 55;noddir[28][28] = 55;noddir[28][29] = 2;noddir[28][30] = 55;noddir[28][31] = 55;noddir[28][32] = 55;noddir[28][33] = 1;noddir[28][34] = 55;noddir[28][35] = 55;noddir[28][36] = 55;
	noddir[29][0]  =  55; noddir[29][1]  =  55; noddir[29][2]  =  55; noddir[29][3] = 55; noddir[29][4] = 55;noddir[29][5] = 55;noddir[29][6] = 55;noddir[29][7] = 55;noddir[29][8] = 55;noddir[29][9] = 55;noddir[29][10] = 55;noddir[29][11] = 55;noddir[29][12] = 55;noddir[29][13] = 55;noddir[29][14] = 55;noddir[29][15] = 55;noddir[29][16] = 55;noddir[29][17] = 55;noddir[29][18] = 55;noddir[29][19] = 55;noddir[29][20] = 55;noddir[29][21] = 55;noddir[29][22] = 55;noddir[29][23] = 55;noddir[29][24] = 55;noddir[29][25] = 55;noddir[29][26] = 55;noddir[29][27] = 55;noddir[29][28] = 0;noddir[29][29] = 55;noddir[29][30] = 2;noddir[29][31] = 1;noddir[29][32] = 55;noddir[29][33] = 55;noddir[29][34] = 55;noddir[29][35] = 55;noddir[29][36] = 55;
	noddir[30][0]  =  55; noddir[30][1]  =  55; noddir[30][2]  =  55; noddir[30][3] = 55; noddir[30][4] = 55;noddir[30][5] = 55;noddir[30][6] = 55;noddir[30][7] = 55;noddir[30][8] = 3;noddir[30][9] = 55;noddir[30][10] = 55;noddir[30][11] = 55;noddir[30][12] = 55;noddir[30][13] = 55;noddir[30][14] = 55;noddir[30][15] = 55;noddir[30][16] = 55;noddir[30][17] = 55;noddir[30][18] = 55;noddir[30][19] = 55;noddir[30][20] = 55;noddir[30][21] = 55;noddir[30][22] = 55;noddir[30][23] = 55;noddir[30][24] = 55;noddir[30][25] = 55;noddir[30][26] = 55;noddir[30][27] = 55;noddir[30][28] = 55;noddir[30][29] = 0;noddir[30][30] = 55;noddir[30][31] = 55;noddir[30][32] = 55;noddir[30][33] = 55;noddir[30][34] = 55;noddir[30][35] = 55;noddir[30][36] = 1;
	noddir[31][0]  =  55; noddir[31][1]  =  55; noddir[31][2]  =  55; noddir[31][3] = 55; noddir[31][4] = 55;noddir[31][5] = 55;noddir[31][6] = 55;noddir[31][7] = 55;noddir[31][8] = 55;noddir[31][9] = 55;noddir[31][10] = 55;noddir[31][11] = 55;noddir[31][12] = 55;noddir[31][13] = 55;noddir[31][14] = 55;noddir[31][15] = 55;noddir[31][16] = 55;noddir[31][17] = 55;noddir[31][18] = 55;noddir[31][19] = 55;noddir[31][20] = 55;noddir[31][21] = 55;noddir[31][22] = 55;noddir[31][23] = 55;noddir[31][24] = 55;noddir[31][25] = 55;noddir[31][26] = 55;noddir[31][27] = 55;noddir[31][28] = 55;noddir[31][29] = 3;noddir[31][30] = 55;noddir[31][31] = 55;noddir[31][32] = 55;noddir[31][33] = 55;noddir[31][34] = 55;noddir[31][35] = 55;noddir[31][36] = 55;
	noddir[32][0]  =  55; noddir[32][1]  =  55; noddir[32][2]  =  55; noddir[32][3] = 55; noddir[32][4] = 55;noddir[32][5] = 55;noddir[32][6] = 55;noddir[32][7] = 55;noddir[32][8] = 55;noddir[32][9] = 55;noddir[32][10] = 55;noddir[32][11] = 55;noddir[32][12] = 55;noddir[32][13] = 55;noddir[32][14] = 55;noddir[32][15] = 55;noddir[32][16] = 55;noddir[32][17] = 55;noddir[32][18] = 55;noddir[32][19] = 55;noddir[32][20] = 55;noddir[32][21] = 55;noddir[32][22] = 55;noddir[32][23] = 55;noddir[32][24] = 55;noddir[32][25] = 55;noddir[32][26] = 55;noddir[32][27] = 55;noddir[32][28] = 55;noddir[32][29] = 55;noddir[32][30] = 55;noddir[32][31] = 55;noddir[32][32] = 55;noddir[32][33] = 0;noddir[32][34] = 55;noddir[32][35] = 55;noddir[32][36] = 55;
	noddir[33][0]  =  55; noddir[33][1]  =  55; noddir[33][2]  =  55; noddir[33][3] = 55; noddir[33][4] = 55;noddir[33][5] = 55;noddir[33][6] = 55;noddir[33][7] = 55;noddir[33][8] = 55;noddir[33][9] = 55;noddir[33][10] = 55;noddir[33][11] = 55;noddir[33][12] = 55;noddir[33][13] = 55;noddir[33][14] = 55;noddir[33][15] = 55;noddir[33][16] = 55;noddir[33][17] = 55;noddir[33][18] = 55;noddir[33][19] = 55;noddir[33][20] = 55;noddir[33][21] = 55;noddir[33][22] = 55;noddir[33][23] = 55;noddir[33][24] = 55;noddir[33][25] = 55;noddir[33][26] = 55;noddir[33][27] = 55;noddir[33][28] = 3;noddir[33][29] = 55;noddir[33][30] = 55;noddir[33][31] = 55;noddir[33][32] = 2;noddir[33][33] = 55;noddir[33][34] = 55;noddir[33][35] = 55;noddir[33][36] = 1;
	noddir[34][0]  =  55; noddir[34][1]  =  55; noddir[34][2]  =  55; noddir[34][3] = 55; noddir[34][4] = 55;noddir[34][5] = 55;noddir[34][6] = 55;noddir[34][7] = 55;noddir[34][8] = 55;noddir[34][9] = 55;noddir[34][10] = 55;noddir[34][11] = 55;noddir[34][12] = 55;noddir[34][13] = 55;noddir[34][14] = 55;noddir[34][15] = 55;noddir[34][16] = 55;noddir[34][17] = 55;noddir[34][18] = 55;noddir[34][19] = 55;noddir[34][20] = 55;noddir[34][21] = 55;noddir[34][22] = 55;noddir[34][23] = 55;noddir[34][24] = 55;noddir[34][25] = 55;noddir[34][26] = 55;noddir[34][27] = 55;noddir[34][28] = 55;noddir[34][29] = 55;noddir[34][30] = 55;noddir[34][31] = 55;noddir[34][32] = 55;noddir[34][33] = 55;noddir[34][34] = 55;noddir[34][35] = 1;noddir[34][36] = 55;
	noddir[35][0]  =  55; noddir[35][1]  =  55; noddir[35][2]  =  55; noddir[35][3] = 55; noddir[35][4] = 55;noddir[35][5] = 55;noddir[35][6] = 55;noddir[35][7] = 55;noddir[35][8] = 55;noddir[35][9] = 55;noddir[35][10] = 55;noddir[35][11] = 55;noddir[35][12] = 55;noddir[35][13] = 55;noddir[35][14] = 55;noddir[35][15] = 55;noddir[35][16] = 55;noddir[35][17] = 55;noddir[35][18] = 55;noddir[35][19] = 55;noddir[35][20] = 55;noddir[35][21] = 55;noddir[35][22] = 55;noddir[35][23] = 55;noddir[35][24] = 55;noddir[35][25] = 0;noddir[35][26] = 55;noddir[35][27] = 55;noddir[35][28] = 55;noddir[35][29] = 55;noddir[35][30] = 55;noddir[35][31] = 55;noddir[35][32] = 55;noddir[35][33] = 55;noddir[35][34] = 3;noddir[35][35] = 55;noddir[35][36] = 2;
	noddir[36][0]  =  55; noddir[36][1]  =  55; noddir[36][2]  =  55; noddir[36][3] = 55; noddir[36][4] = 55;noddir[36][5] = 55;noddir[36][6] = 55;noddir[36][7] = 55;noddir[36][8] = 55;noddir[36][9] = 55;noddir[36][10] = 55;noddir[36][11] = 55;noddir[36][12] = 55;noddir[36][13] = 55;noddir[36][14] = 55;noddir[36][15] = 55;noddir[36][16] = 55;noddir[36][17] = 55;noddir[36][18] = 55;noddir[36][19] = 55;noddir[36][20] = 55;noddir[36][21] = 55;noddir[36][22] = 55;noddir[36][23] = 55;noddir[36][24] = 55;noddir[36][25] = 55;noddir[36][26] = 55;noddir[36][27] = 55;noddir[36][28] = 55;noddir[36][29] = 55;noddir[36][30] = 2;noddir[36][31] = 55;noddir[36][32] = 55;noddir[36][33] = 3;noddir[36][34] = 55;noddir[36][35] = 0;noddir[36][36] = 55;

	
	dista[0]=55;dista[1]=55;dista[2]=55;dista[3]=55;dista[4]=55;dista[5]=55;dista[6]=55;dista[7]=55;dista[8]=55;dista[9]=55;dista[10]=55;dista[11]=55;dista[12]=55;dista[13]=55;dista[14]=55;dista[15]=55;dista[16]=55;dista[17]=55;dista[18]=55;dista[19]=55;dista[20]=55;dista[21]=55;dista[22]=55;dista[23]=55;dista[24]=55;dista[25]=55;dista[26]=55;dista[27]=55;dista[28]=55;dista[29]=55;dista[30]=55;dista[31]=55;dista[32]=55;dista[33]=55;dista[34]=55;dista[35]=55;dista[36]=55;
   parent[0]=0;parent[1]=1;parent[2]=2;parent[3]=3;parent[4]=4;parent[5]=5;parent[6]=6;parent[7]=7;parent[8]=8;parent[9]=9;parent[10]=10;parent[11]=11;parent[12]=12;parent[13]=13;parent[14]=14;parent[15]=15;parent[16]=16;parent[17]=17;parent[18]=18;parent[19]=19;parent[20]=20;parent[21]=21;parent[22]=22;parent[23]=23;parent[24]=24;parent[25]=25;parent[26]=26;parent[27]=27;parent[28]=28;parent[29]=29;parent[30]=30;parent[31]=31;parent[32]=32;parent[33]=33;parent[34]=34;parent[35]=35;parent[36]=36;
	end
	
reg [20:0] pl_counter=0;
reg pl_rst=0;
reg pl_trig=0;


always@(*) begin 
		pl_rst<=(pl_counter<120)?0:1;
		end

always@(posedge enpl or  posedge pl_rst) begin
		if(pl_rst) begin
				pl_trig<=0;
				end
		else begin
				pl_trig<=1;
				end
end

always@(posedge clk or posedge pl_rst) begin
		if(pl_rst) begin
			pl_counter<=0;
			end
		else begin
			  if(pl_trig) begin
					pl_counter<=pl_counter+1;
					end
      end
end


always @(posedge clk) begin
	
	
	case(state)
		
			NEAR : begin
					dista[stno]<=0;
					 
					if(j==37) begin
							state<=PATH;
							end
				   else if(i==37) begin
					          j<=j+1;
								 state<=ADJ;
								 i<=0;
								 min<=55;
								 end
					else begin
						  if(~vis[i] && dista[i]<min) begin
									nearest<=i;
									i<=i+1;  
									state<=NEAR;
									min<=dista[i];
							      end
							else begin
									i<=i+1;
									state<=NEAR;
							     end
						   end
					 end
			ADJ : begin
					vis[nearest]<=1;
					if(i==37) begin
								state<=NEAR;
								i<=0;
								
								end
							else if((mapbot[nearest][i]<55) && (dista[i]>(dista[nearest]+mapbot[nearest][i])) && (~vis[i])) begin
											dista[i]<=dista[nearest]+mapbot[nearest][i];
											i<=i+1;
											state<=ADJ;
											parent[i]<=nearest;
											end
							else begin
								i<=i+1;
								state<=ADJ;
								end
							
							
					end
			PATH: begin
					if(temp==stno) begin
						state<=GETDIR;
						path[k]<= stno;
						curnod<=path[k];
						nxtnod<=path[k-1];
						
						end
					else begin
						path[k]<= temp;
						temp<=parent[temp];
						k<=k+1;	
						end
					end
			 GETDIR: begin
					
					if(k==0) begin
						state<=STOP;
						length<=le;
						end
					else begin
					   curnod<=path[k];
						nxtnod<=path[k-1];
						k<=k-1;
						state<=ASGDIR;
					end
					end
			ASGDIR: begin
						nxtdir<=noddir[curnod][nxtnod];
						state<=PLANDIR;
						end
			PLANDIR: begin
							if((nxtdir-curdir)==2) begin
								plan[le]<=2;
								curdir<=1;
								end
							else begin
								plan[le]<=nxtdir-curdir;
								curdir<=nxtdir;
								end
							
							state<=GETDIR;
							le<=le+1;
							
						end
			
			STOP: begin
						
						j<=0;
						le<=0;
						k<=0;
						i<=0;
						vis<=0;
						min<=55;
						dista[dcount]<=55;
						dcount<=dcount+1;
						
						
   
						if(pl_trig==1) begin
							state<=REINIT;
							
						end
						else begin
							state<=STOP;
							end
					end
			REINIT: begin
						dista[stno]<=0;
						temp<=enno;
						length<=0;
						state<=NEAR;
						curdir<=orientation;
						end
			default : begin
						    state<=state;
					       end
		endcase
	end	

assign sl0=plan[0];assign sl1=plan[1];assign sl2=plan[2];assign sl3=plan[3];assign sl4=plan[4];assign sl5=plan[5];assign sl6=plan[6];assign sl7=plan[7];assign sl8=plan[8];assign sl9=plan[9];assign sl10=plan[10]; assign sl11=plan[11];assign sl12=plan[12];assign sl13=plan[13];assign sl14=plan[14];assign sl15=plan[15];assign sl16=plan[16];assign sl17=plan[17];
assign sl18=plan[18];assign sl19=plan[19];assign sl20=plan[20];assign sl21=plan[21];assign sl22=plan[22];assign sl23=noddir[0][0];assign sl24=mapbot[0][0];assign sl25=plan[25];assign sl26=plan[26];assign sl27=plan[27];assign sl28=plan[28];assign sl29=plan[29];assign sl30=plan[30];assign sl31 =plan[31];assign sl32 =plan[32];assign sl33 =plan[33];assign sl34 =plan[34];assign sl35 =plan[35];assign sl36 =plan[36];
assign len=length;
  endmodule
