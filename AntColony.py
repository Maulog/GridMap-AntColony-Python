import random
from math import sqrt
import numpy as np
from matplotlib import colors
import matplotlib.pyplot as plt
from Utils import load_argparse

class Map():
    '''
    :param:地图类
    :param:使用时需要传入行列两个参数 再实例化
    '''

    def __init__(self,row,col):
        '''
        :param:row::行
        :param:col::列
        '''
        self.row = row
        self.col = col
        self.area = row*col
        self.data = [[0 for i in range(self.col)] for j in range(self.row)]
        
    def getXy(self,id,col):
        x=id%col
        y=id//col
        return [y,x]

    def getId(self,x,y,col):
        id=y*col+x
        return id

    def mapObstacle(self,num):
        '''
        :param:num:地图障碍物数量
        :return:返回包含障碍物的地图数据
        '''
        self.obsnum = num
        NumList=np.array(range(0,self.area))
        NumList=NumList.reshape(self.row,self.col)
        #给右下边框留出空 
        numList=NumList[0:self.row-4-1,0:self.col-4-1].flatten().tolist()
        
        random.shuffle(numList)
        randomList=numList[0:self.obsnum]
        randomList.sort()
        rLxy=[]
        for id in randomList:
            rLxy.append(self.getXy(id,self.col))

        def creatObs(point):#在4x4的范围生成障碍
            n=random.randint(6,16)
            rl=list(range(16))[:n]
            random.shuffle(rl)            
            for i in rl:
                [deltaY,deltaX]=self.getXy(i,4)
                self.data[point[1]+deltaY][point[0]+deltaX]=1
            self.data[0][0]=0
            self.data[self.row-1][self.col-1]=0

        for point in rLxy:
            creatObs(point)
        
        return self.data
    
    def mapShow(self):
        cmap = colors.ListedColormap(['none', 'black', 'white', 'magenta', 'yellow', 'cyan', 'green', 'red', 'blue'])
        plt.imshow(self.data, cmap=cmap, interpolation='nearest', vmin=0, vmax=7)
        plt.show()

        
class ACO():
    def __init__(self,antNum,iterMax,rho,Q,alpha,beta,map):
        self.antNum=antNum
        self.iterMax=iterMax
        self.rho=rho
        self.Q=Q
        self.alpha=alpha
        self.beta=beta
        self.map=np.array(map)
        self.mapCol=self.map.shape[1]
        self.mapRow=self.map.shape[0]
        self.area=self.mapCol*self.mapRow
        self.start=[0,self.mapRow-1]    #起点终点坐标
        self.end=[self.mapCol-1,0]

        self.shortestRoute=[]           #保存每次迭代后的短距离的序号
        self.shortestDistance=[]        #保存每次迭代的最短距离
        self.currentRoute=[]            #保存当前行走的路径
        self.pheromonetable=np.ones(self.map.shape)-self.map
        #将障碍物处的信息素浓度设为0，在概率计算中为0不影响计算结果

    def getDistance(self,p1,p2):
        #p1 p2为列表[x1 y1][x2 y2]
        dist=sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)
        if dist==0:
            dist=1e-4                              #若探测点为终点，防止除零
        return dist
    
    def antMove(self,currentLoc):                  #轮盘赌法确定下一步行走
        if currentLoc==self.start:
            self.currentRoute.append(currentLoc)
        
        def detect(point):
            #此函数约束地图的边界
            #point为[x,y]
            #0为不可通行，1为可以通行
            dectPhase=[1,1,1,1]
            if point[1]-1<0:             #上边界
                dectPhase[0]=0
            if point[1]+1>self.mapRow-1: #下边界
                dectPhase[1]=0
            if point[0]-1<0:             #左边界
                dectPhase[2]=0
            if point[0]+1>self.mapCol-1: #右边界
                dectPhase[3]=0
            return dectPhase
        availableLoc=detect(currentLoc)

        if availableLoc[0]==1:            #计算下一步概率
            Eta=1/self.getDistance([currentLoc[0],currentLoc[1]-1],self.end)
            Tau=self.pheromonetable[currentLoc[1]-1][currentLoc[0]]
            up=pow(Tau,self.alpha)*pow(Eta,self.beta)
        else:
            up=0
        if availableLoc[1]==1:
            Eta=1/self.getDistance([currentLoc[0],currentLoc[1]+1],self.end)
            Tau=self.pheromonetable[currentLoc[1]+1][currentLoc[0]]
            down=pow(Tau,self.alpha)*pow(Eta,self.beta)
        else:
            down=0
        if availableLoc[2]==1:
            Eta=1/self.getDistance([currentLoc[0]-1,currentLoc[1]],self.end)
            Tau=self.pheromonetable[currentLoc[1]][currentLoc[0]-1]
            left=pow(Tau,self.alpha)*pow(Eta,self.beta)
        else:
            left=0
        if availableLoc[3]==1:
            Eta=1/self.getDistance([currentLoc[0]+1,currentLoc[1]],self.end)
            Tau=self.pheromonetable[currentLoc[1]][currentLoc[0]+1]
            right=pow(Tau,self.alpha)*pow(Eta,self.beta)
        else:
            right=0
        allChoice=up+down+left+right
        p1=up/allChoice
        p2=down/allChoice
        p3=left/allChoice
        p4=right/allChoice

        Roulette=random.random()        #轮盘赌法
        if Roulette>=0 and Roulette<p1:
            self.currentRoute.append([currentLoc[0],currentLoc[1]-1])
        elif Roulette>=p1 and Roulette<p1+p2:
            self.currentRoute.append([currentLoc[0],currentLoc[1]+1])
        elif Roulette>=p1+p2 and Roulette<p1+p2+p3:
            self.currentRoute.append([currentLoc[0]-1,currentLoc[1]])
        elif Roulette>=p1+p2+p3 and Roulette<p1+p2+p3+p4 and Roulette<=1:
            self.currentRoute.append([currentLoc[0]+1,currentLoc[1]])
        
        self.cLoc=self.currentRoute[-1]  #更新当前坐标

        if self.cLoc==self.end:
            return False
        else:
            return True

    def updatePheromonetablek(self,route):
        self.deltaPheromonetable=np.zeros(self.map.shape)
        L=len(route)
        avgDeltaTauk=self.Q/L
        deltaTauk=np.zeros(self.map.shape)
        for step in route:
            deltaTauk[step[1]][step[0]]=avgDeltaTauk
        self.deltaPheromonetable+=deltaTauk

    def updatePheromonetable(self,deltaPheromonetable):
        self.pheromonetable=(1-self.rho)*self.pheromonetable+self.rho*deltaPheromonetable
    
    def acoCalculate(self):
        for iter in range(self.iterMax):
            self.cLoc=self.start
            for k in range(self.antNum):
                while self.antMove(self.cLoc):
                    pass
                self.updatePheromonetablek(self.currentRoute)
                if k==0 and iter==0:
                    self.shortestRoute=self.currentRoute
                    shortestDistancek=len(self.currentRoute)
                if len(self.currentRoute)<shortestDistancek:
                    self.shortestRoute=[]
                    self.shortestRoute=self.currentRoute
                    shortestDistancek=len(self.currentRoute)
                # print(self.currentRoute)
                self.currentRoute=[]
                self.cLoc=self.start
            self.updatePheromonetable(self.deltaPheromonetable)
            self.shortestDistance.append(shortestDistancek)
            print("[{}/{}] now the shortest route length is {}"
                    .format(iter+1,self.iterMax,self.shortestDistance[-1]))

        

    def resultShow(self):
        plt.figure(0)
        for step in self.shortestRoute:
            self.map[step[1]][step[0]]=3
        self.map[self.start[1]][self.start[0]]=5
        self.map[self.end[1]][self.end[0]]=6
        
        cmap = colors.ListedColormap(['white', 'black', 'magenta', 'orange', 'cyan', 'green', 'red', 'blue'])
        plt.title("Map")
        plt.xlabel("x")
        plt.ylabel("y")
        plt.imshow(self.map, cmap=cmap, interpolation='nearest', vmin=0, vmax=7)
        plt.savefig("./res/map.png")


        plt.figure(1)
        iterNum=np.arange(self.iterMax)
        plt.title("the shortest distance")
        plt.xlabel("iter")
        plt.ylabel("distance")
        plt.plot(iterNum,np.array(self.shortestDistance),color='r',linestyle='-')
        plt.savefig("./res/shortest_distance.png")
        
        plt.show()




if __name__=='__main__':
    configs=load_argparse()
    map=Map(configs.map.row,configs.map.col)
    map.mapObstacle(configs.map.obsNum)
    
    aco=ACO(configs.aco.antNum,
            configs.aco.iterMax,
            configs.aco.rho,
            configs.aco.Q,
            configs.aco.alpha,
            configs.aco.beta,
            map.data)
    
    aco.acoCalculate()
    aco.resultShow()


