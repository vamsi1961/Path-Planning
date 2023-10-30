from maze import maze,agent,COLOR


def DFS(m):

    # starting point is initial point bottom left point

    # stacks are required in python we can use list    
    start = (m.rows,m.cols)
    

    # create 2 lists  explored and frontier 

    # explored list is to add coordinates of points which are explored.
    explored = [start]

    # frontier list is to add coordinates of points which are to be explored i.e next possible childs    
    frontier = [start]
    

    # create a dictionary to add cell and its parent viewed from starting 
    # when travelling key is cell value is it's next cell
    dfsPAth = {}
    

    while len(frontier) > 0:
        # when therre are cells to explore 
        CurrCell = frontier.pop()

        # get cell from frontier and pop it from frontier

        if CurrCell == (1,1):
                break
        
        # Prioritize how to move in maze
        for d in "ESNW":
            
            
            # Checking is there any way to move in that particular direction
            if m.maze_map[CurrCell][d]==True:
                

                if d == "E":
                    ChildCell = (CurrCell[0] , CurrCell[1] + 1)

                elif d == "S":
                    ChildCell = (CurrCell[0]+1 , CurrCell[1])

                elif d == "N":
                    ChildCell = (CurrCell[0]-1 , CurrCell[1])

                elif d == "W":
                    ChildCell = (CurrCell[0] , CurrCell[1]-1)

                if ChildCell in explored:
                    continue

                # after the cell is explored add it into explored and frontier
                explored.append(ChildCell)
                frontier.append(ChildCell)

                # Append all cell and it's corresponding child cell into a dictionary when inverted gives a proper path

                dfsPAth[ChildCell] = CurrCell

                # make a dictionary and store child cell and its corresponding cell


    fwdPAth = {}
    cell = (1,1)
    # log all the data in new dictionary and display in maze
    while cell != start:
        # get a dictionary of only path cells and corresponding next cells
        fwdPAth[dfsPAth[cell]] = cell
        cell = dfsPAth[cell]

    return fwdPAth

if __name__ == '__main__':

    m = maze(10,10)
    # get the maximum number of multiple paths
    m.CreateMaze(loopPercent=100)

    # get dictionary of cells and corresponding child cells
    
    path = DFS(m)
    # create an agent
    # footprints = true to see 
    # the complete path travels by agent
    a = agent(m,footprints=True)
    
    # input we pass the dictionary with key as the agent and the value as the path travelled by the agent.
    m.tracePath({a:path})
    
    m.run()



            
                
