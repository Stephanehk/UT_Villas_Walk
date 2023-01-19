

def segment(data):
    ptr = 0
    
    segments = []
    while ptr < len(data):
        while (ptr < len(data) and data[ptr] != "("):
            ptr += 1
        
        if ptr >= len(data):
            break

        brac_count = 0
        current_str = ""
        is_first = True

        while ((brac_count != 0 and ptr < len(data)) or is_first):
            is_first = False
            if (data[ptr] == "("):
                brac_count +=1
            elif (data[ptr] == ")"):
                brac_count -=1
            
            current_str += data[ptr]
            ptr+=1
        
        if (brac_count == 0):
            segments.append(current_str)
    return segments


def parse(fp):
    with open(fp, 'r') as file:
        data = file.read()

    if ("mypos" in data):
        assert False
    
    segments = segment(data)
    for seg in segments:
        print(seg)

parse("/home/stephane/ros2_ws/sparkmonitor.log")