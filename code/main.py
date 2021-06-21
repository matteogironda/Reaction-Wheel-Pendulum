import controller

c = controller.controller() 

def run():

    i = 0  
    while i < 100:
        state = c.get_state()
        c.control(state)
        i += 1
    c.turn_off(c)

if __name__ == "__main__":
    
    run()
    controller.i2c_test()
    
