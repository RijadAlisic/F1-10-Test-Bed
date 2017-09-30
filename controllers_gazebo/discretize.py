import numpy as np
from tulip import transys, spec, synth

def add_states(names,system):
    system.states.add_from(names)

def add_transitions(from_state,to_states,system):
    system.transitions.add_comb(from_state,to_states)

def road(system,startpoint_x,startpoint_y,endpoint_x,endpoint_y,road_width,lanes=1):
    lane_width=road_width/lanes
    road_length=np.sqrt((startpoint_x-endpoint_x)**2+(startpoint_y-endpoint_y)**2)
    
    dx=0
    dy=0
    if((startpoint_x-endpoint_x)**2>0):
        direction='horizontal'
        dx=np.sign(endpoint_x-startpoint_x)*1.25
        width_x=np.abs(dx)
        width_y=road_width/lanes
        x=startpoint_x#+road_width/2*np.sign(endpoint_x-startpoint_x)-dx/2
        y=startpoint_y
    elif((startpoint_y-endpoint_y)**2>0):
        direction='vertical'
        dy=np.sign(endpoint_y-startpoint_y)*1.25
        width_y=np.abs(dy)
        width_x=road_width/lanes
        x=startpoint_x
        y=startpoint_y#+road_width/2*np.sign(endpoint_y-startpoint_y)-dy/2
    else:
        print 'Error, road no stretch'
        return 0
    
    dphi=0
    num_angles=1
    
    num_states_main=int(np.floor((road_length-road_width)/np.max([np.abs(dx),np.abs(dy)])))
    for i in range(0,num_states_main):
        phi=int(np.arctan2(dy,dx)*4/np.pi)-dphi
        x+=dx
        y+=dy
        for j in range(0,num_angles):
            string_state='Xequal'+str(x)+'spaceYequal'+str(y)+'spacephiequal'+str(np.mod(phi+dphi*j,8))
            add_states([string_state],system)
            num_states=len(system.states)
            
            for k in range(0,num_states):
                if system.states()[k] is string_state:
                    curr_state=k
            #print [system.states()[curr_state]]
            num_transitions=0
            add_transitions([system.states()[curr_state]],[system.states()[curr_state]],system)
            for k in range(0,num_states):
                param=system.states()[k].split('space')
                temp_x=float(param[0].split('equal')[1])
                temp_y=float(param[1].split('equal')[1])
                temp_phi=float(param[2].split('equal')[1])
                #print temp_x-x,x,temp_x
                #print temp_y-y,y,temp_y
                #print temp_phi-phi,phi,temp_phi
                #print curr_state,phi+j*dphi
                
                
                if(direction is 'horizontal' and np.abs(np.mod(phi+dphi*j,8)-temp_phi)<=1.1):
                    if x-temp_x>0.1 and dx>0:
                        if np.abs(x-temp_x)<np.abs(dx*1.5) and np.abs(y-temp_y)<0.001:
                            add_transitions([system.states()[k]],[system.states()[curr_state]],system)
                            num_transitions+=1
                    elif x-temp_x<-0.1 and dx<0:
                        if np.abs(x-temp_x)<np.abs(dx*1.5) and np.abs(y-temp_y)<0.001:
                            add_transitions([system.states()[k]],[system.states()[curr_state]],system)
                            num_transitions+=1
                elif(direction is 'vertical' and np.abs(np.mod(phi+dphi*j,8)-temp_phi)<=1.1):
                    if y-temp_y>0.1 and dy>0:
                        #print 'trans'
                        if np.abs(y-temp_y)<np.abs(dy*1.5) and np.abs(x-temp_x)<0.001:
                            add_transitions([system.states()[k]],[system.states()[curr_state]],system)
                            num_transitions+=1
                    elif y-temp_y<-0.1 and dy<0:
                        if np.abs(y-temp_y)<np.abs(dy*1.5) and np.abs(x-temp_x)<0.001:
                            add_transitions([system.states()[k]],[system.states()[curr_state]],system)
                            num_transitions+=1
            #print num_transitions
        if lanes>1:
          print 'multiple lanes not supported yet'  

def corner(system,start_x,start_y,first_direction,end_x,end_y,lanes=1):
    #not working!
    x=start_x
    y=start_y
    dphi=0
    if(start_x is end_x):
        y=(end_y+start_y)/2
    elif(start_y is end_y):
        x=(end_x+start_x)/2
    elif first_direction is 'horizontal':
        x=end_x
        last_direction='vertical'
    elif first_direction is 'vertical':
        y=end_y
        last_direction='horizontal'
    phi=int(np.arctan2(end_y-start_y,end_x-start_x))-dphi
    for i in range(0,1):
        string_state='Xequal'+str(x)+'spaceYequal'+str(y)+'spacephiequal'+str(np.mod(phi+dphi*i,8))
        #print string_state
        add_states([string_state],system)
        num_states=len(system.states())
        for k in range(0,num_states):
            if system.states()[k] is string_state:
                curr_state=k
        for j in range(0,1):
            try:
                print ['Xequal'+str(start_x)+'spaceYequal'+str(start_y)+'spacephiequal'+str(np.mod(phi+dphi*j,8))],[string_state]
                add_transitions(['Xequal'+str(start_x)+'spaceYequal'+str(start_y)+'spacephiequal'+str(np.mod(phi+dphi*j,8))],[string_state],system)
                
            except:
                print 'not passed'
                pass
            try:
                add_transitions([string_state],['Xequal'+str(end_x)+'spaceYequal'+str(end_y)+'spacephiequal'+str(np.mod(phi+dphi*j,8))],system)
            except:
                pass



def full_road_ex():
    import discretize
    from tulip import transys
    sys=transys.FTS()
    road(sys,0,0,0,5,0.4)
    road(sys,0,5,5,5,0.4)
    road(sys,5,5,5,0,0.4)
    road(sys,5,0,0,0,0.4)
    add_states(['Xequal0spaceYequal0spacephiequal'+str(3)],sys)
    #print sys.states()
    add_transitions(['Xequal0spaceYequal3.75spacephiequal'+str(2)],['Xequal1.25spaceYequal5spacephiequal'+str(0)],sys)
    
    add_transitions(['Xequal3.75spaceYequal5spacephiequal'+str(0)],['Xequal5spaceYequal3.75spacephiequal'+'6'],sys)
    
    add_transitions(['Xequal5spaceYequal1.25spacephiequal'+'6'],['Xequal3.75spaceYequal0spacephiequal'+str(4)],sys)
    
    add_transitions(['Xequal1.25spaceYequal0spacephiequal'+str(4)],['Xequal0spaceYequal1.25spacephiequal'+str(2)],sys)
    
    add_transitions(['Xequal0spaceYequal0spacephiequal'+str(3)],['Xequal0spaceYequal0spacephiequal'+str(3)],sys)
    
    add_transitions(['Xequal0spaceYequal0spacephiequal'+str(3)],['Xequal0spaceYequal1.25spacephiequal'+str(2)],sys)
    
    add_states(['Xequal1.25spaceYequal2.5spacephiequal'+str(0)],sys)
    add_transitions(['Xequal0spaceYequal1.25spacephiequal'+str(2)],['Xequal1.25spaceYequal2.5spacephiequal'+str(0)],sys)
    add_transitions(['Xequal1.25spaceYequal2.5spacephiequal'+str(0)],['Xequal1.25spaceYequal2.5spacephiequal'+str(0)],sys)
    add_transitions(['Xequal1.25spaceYequal2.5spacephiequal'+str(0)],['Xequal0spaceYequal1.25spacephiequal'+str(2)],sys)

    add_states(['Xequalmmm1.25spaceYequal2.5spacephiequal'+str(4)],sys)
    add_transitions(['Xequal0spaceYequal1.25spacephiequal'+str(2)],['Xequalmmm1.25spaceYequal2.5spacephiequal'+str(4)],sys)
    add_transitions(['Xequalmmm1.25spaceYequal2.5spacephiequal'+str(4)],['Xequalmmm1.25spaceYequal2.5spacephiequal'+str(4)],sys)
    add_transitions(['Xequalmmm1.25spaceYequal2.5spacephiequal'+str(4)],['Xequal0spaceYequal1.25spacephiequal'+str(2)],sys)

    add_states(['Xequal2.5spaceYequal6.25spacephiequal'+str(2)],sys)
    add_transitions(['Xequal1.25spaceYequal5spacephiequal'+str(0)],['Xequal2.5spaceYequal6.25spacephiequal'+str(2)],sys)
    add_transitions(['Xequal2.5spaceYequal6.25spacephiequal'+str(2)],['Xequal2.5spaceYequal6.25spacephiequal'+str(2)],sys)
    add_transitions(['Xequal2.5spaceYequal6.25spacephiequal'+str(2)],['Xequal1.25spaceYequal5spacephiequal'+str(0)],sys)

    add_states(['Xequal2.5spaceYequal3.75spacephiequal'+str(6)],sys)
    add_transitions(['Xequal1.25spaceYequal5spacephiequal'+str(0)],['Xequal2.5spaceYequal3.75spacephiequal'+str(6)],sys)
    add_transitions(['Xequal2.5spaceYequal3.75spacephiequal'+str(6)],['Xequal2.5spaceYequal3.75spacephiequal'+str(6)],sys)
    add_transitions(['Xequal2.5spaceYequal3.75spacephiequal'+str(6)],['Xequal1.25spaceYequal5spacephiequal'+str(0)],sys)

    add_states(['Xequal6.25spaceYequal2.5spacephiequal'+str(0)],sys)
    add_transitions(['Xequal5spaceYequal3.75spacephiequal'+str(6)],['Xequal6.25spaceYequal2.5spacephiequal'+str(0)],sys)
    add_transitions(['Xequal6.25spaceYequal2.5spacephiequal'+str(0)],['Xequal6.25spaceYequal2.5spacephiequal'+str(0)],sys)
    add_transitions(['Xequal6.25spaceYequal2.5spacephiequal'+str(0)],['Xequal5spaceYequal3.75spacephiequal'+str(6)],sys)

    add_states(['Xequal3.75spaceYequal2.5spacephiequal'+str(4)],sys)
    add_transitions(['Xequal5spaceYequal3.75spacephiequal'+str(6)],['Xequal3.75spaceYequal2.5spacephiequal'+str(4)],sys)
    add_transitions(['Xequal3.75spaceYequal2.5spacephiequal'+str(4)],['Xequal3.75spaceYequal2.5spacephiequal'+str(4)],sys)
    add_transitions(['Xequal3.75spaceYequal2.5spacephiequal'+str(4)],['Xequal5spaceYequal3.75spacephiequal'+str(6)],sys)

    add_states(['Xequal2.5spaceYequal1.25spacephiequal'+str(2)],sys)
    add_transitions(['Xequal3.75spaceYequal0spacephiequal'+str(4)],['Xequal2.5spaceYequal1.25spacephiequal'+str(2)],sys)
    add_transitions(['Xequal2.5spaceYequal1.25spacephiequal'+str(2)],['Xequal2.5spaceYequal1.25spacephiequal'+str(2)],sys)
    add_transitions(['Xequal2.5spaceYequal1.25spacephiequal'+str(2)],['Xequal3.75spaceYequal0spacephiequal'+str(4)],sys)

    add_states(['Xequal2.5spaceYequalmmm1.25spacephiequal'+str(6)],sys)
    add_transitions(['Xequal3.75spaceYequal0spacephiequal'+str(4)],['Xequal2.5spaceYequalmmm1.25spacephiequal'+str(6)],sys)
    add_transitions(['Xequal2.5spaceYequalmmm1.25spacephiequal'+str(6)],['Xequal2.5spaceYequalmmm1.25spacephiequal'+str(6)],sys)
    add_transitions(['Xequal2.5spaceYequalmmm1.25spacephiequal'+str(6)],['Xequal3.75spaceYequal0spacephiequal'+str(4)],sys)
    
    sys.atomic_propositions.add_from({'pSpace'})
    sys.states.add('Xequal1.25spaceYequal2.5spacephiequal'+str(0), ap={'pSpace'})
    sys.states.add('Xequalmmm1.25spaceYequal2.5spacephiequal'+str(4), ap={'pSpace'})
    sys.states.add('Xequal2.5spaceYequal6.25spacephiequal'+str(2), ap={'pSpace'})
    sys.states.add('Xequal2.5spaceYequal3.75spacephiequal'+str(6), ap={'pSpace'})
    sys.states.add('Xequal6.25spaceYequal2.5spacephiequal'+str(0), ap={'pSpace'})
    sys.states.add('Xequal3.75spaceYequal2.5spacephiequal'+str(4), ap={'pSpace'})
    sys.states.add('Xequal2.5spaceYequal1.25spacephiequal'+str(2), ap={'pSpace'})
    sys.states.add('Xequal2.5spaceYequalmmm1.25spacephiequal'+str(6), ap={'pSpace'})
    
    return sys

def synth_ex():
    sys=full_road_ex()
    variables=''
    for i in range(0,len(sys.states())):
	#print sys.states()[i],i
        sys.atomic_propositions.add_from({'x'+str(i)})
        sys.states[sys.states()[i]]['ap'].add('x'+str(i))
        variables+=' && ((X (x'+str(i)+')) <-> x'+str(i)+')'
    variables=variables[4:]
    #print variables
    #sys.atomic_propositions.add_from({'home', 'lot'})
    #sys.states.add('Xequal0spaceYequal0spacephiequal3', ap={'home'})
    #sys.states.add('Xequal5spaceYequal5spacephiequal7', ap={'lot'})
    sys.states.initial.add('Xequal0spaceYequal0spacephiequal3')
    env_vars = {'park','frontclear'}
    env_init = set()
    env_prog = {'frontclear'}
    env_safe = set()
    
    sys_vars = {'pReach'}
    sys_init = {'pReach'}
    sys_prog = {'x0','x10'}            
    sys_safe = {'((X (pReach)) <-> ((pSpace) || (pReach && !park)))','((!frontclear) -> ('+variables+'))'} 
    sys_prog |={'pReach'}
    specs = spec.GRSpec(env_vars, sys_vars, env_init, sys_init, env_safe, sys_safe, env_prog, sys_prog)
    specs.moore = True
    specs.qinit = '\E \A'
    ctrl = synth.synthesize('omega',specs, sys=sys)
    assert ctrl is not None, 'unrealizable'
    #ctrl.save('discrete.pdf')
    return sys,ctrl
