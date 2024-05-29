import json
import time
from tkinter import *
from PIL import Image, ImageTk
from itertools import zip_longest
import numpy as np

prev_x = None
prev_y = None
res_points = None
res_velocity = None
res_coeff = None
Plane = []
point1 = {}
point2 = {}
ImitationRequest = ''
ImitationResponse = ''



def requestPointToNPPoint(p):
    return np.array([[p['x']], [p['y']]], np.float64)

def draw_sight_line(canvas, missile_pos, target_pos, line_id=None, color="black"):
    """
    Draws a sight line from missile position to target position on the canvas.
    If line_id is provided, it will update the existing line, otherwise it will create a new line.
    """
    x1, y1 = missile_pos
    x2, y2 = target_pos
    if line_id is not None:
        canvas.coords(line_id, x1, y1, x2, y2)
        return line_id
    else:
        return canvas.create_line(x1, y1, x2, y2, fill=color, dash=(4, 4))


def start():
    global res_points
    global res_velocity
    global res_coeff
    global Plane
    global point1
    global point2
    global isUsualHit
    global isFuzzyHit

    res_points = points.get()
    res_points = int(res_points)

    res_velocity = velocity.get()
    res_velocity = int(res_velocity)

    defuzz = "Centroid"
    inference = "Max-Prod"

    res_coeff = 3
    res_coeff = int(res_coeff)

    AircraftPoints = {"AircraftPoints": Plane,
                      "Missiles": {"Defuzzification": defuzz, "Direction": point2, "Inference": inference,
                                   "LaunchPoint": point1, "PropCoeff": res_coeff, "VelocityModule": res_velocity},
                      "StepsCount": res_points}

    ImitationRequest = json.dumps(AircraftPoints, ensure_ascii=False)
    import trajectory
    tr = trajectory.TrajectoryGenerator(ImitationRequest)
    ImitationResponse = tr.response_s
    data = json.loads(ImitationResponse)

    curvesBasicPoints = np.hstack(tuple(map(requestPointToNPPoint, data['AircraftTrajectory'])))
    for i in range(res_points - 5):
        x = curvesBasicPoints[0][i]
        y = curvesBasicPoints[1][i]
        if i % 3 == 0:
            canvas.create_oval(x - 2.0, y + 2.0, x + 2.0, y - 2.0, outline="BLACK", fill="BLACK")

    settings = data['UsualMissile']
    curvesUsual = np.hstack(tuple(map(requestPointToNPPoint, settings['Trajectory'])))
    UsualP = np.shape(curvesUsual)
    for i in range(UsualP[1] - 1):
        x = curvesUsual[0][i]
        y = curvesUsual[1][i]
        if i % 3 == 0:
            canvas.create_oval(x - 1.0, y + 1.0, x + 1.0, y - 1.0, outline="BLUE", fill="BLUE")
    UsualHit = settings['IsHit']
    usual_arr = settings['CurrentDistance']
    usual_bearings = settings['CurrentBearing']

    settings = data['FuzzyMissile']
    curvesFuzzy = np.hstack(tuple(map(requestPointToNPPoint, settings['Trajectory'])))
    FuzzyP = np.shape(curvesFuzzy)
    for i in range(FuzzyP[1] - 1):
        x = curvesFuzzy[0][i]
        y = curvesFuzzy[1][i]
        if i % 3 == 0:
            canvas.create_oval(x - 1.0, y + 1.0, x + 1.0, y - 1.0, outline="RED", fill="RED")
    FuzzyHit = settings['IsHit']
    fuzzy_arr = settings['CurrentDistance']
    fuzzy_bearings = settings['CurrentBearing']

    if UsualHit:
        hitUsual.config(text='True', bg='green')
    else:
        hitUsual.config(text='False', bg='red')

    if FuzzyHit:
        hitFuzz.config(text='True', bg='green')
    else:
        hitFuzz.config(text='False', bg='red')

    img_plane2 = ImageTk.PhotoImage(Image.open("img/plane.png").resize((50, 50)) )
    plane_id = canvas.create_image(0, 0, image=img_plane2, anchor="center")
    oval2 = canvas.create_oval(0, 0, 0, 0, fill="BLUE")
    oval3 = canvas.create_oval(0, 0, 0, 0, fill="RED")

    FuzzyFlag = False
    UsualFlag = False

    # print(usual_bearings)
    # print(fuzzy_bearings)

    combined_arrays = zip_longest(usual_arr, fuzzy_arr, usual_bearings, fuzzy_bearings)
    usual_line_id = None
    fuzzy_line_id = None

    for i, (dist1, dist2, bearing1, bearing2) in enumerate(combined_arrays):
        x1 = curvesBasicPoints[0][i]
        y1 = curvesBasicPoints[1][i]
        canvas.coords(plane_id, x1, y1)
        if dist1 is not None:
            distanceUsual.config(text=f"{usual_arr[i]}")
            bearingUsual.config(text=f"{bearing1}°")
        if dist2 is not None:
            distanceFuzz.config(text=f"{fuzzy_arr[i]}")
            bearingFuzz.config(text=f"{bearing2}°")
        time.sleep(0.02)
        window.update()

        if i < UsualP[1]:
            x2 = curvesUsual[0][i]
            y2 = curvesUsual[1][i]
            canvas.coords(oval2, x2 - 5.0, y2 + 5.0, x2 + 5.0, y2 - 5.0)
            usual_line_id = draw_sight_line(canvas, (x2, y2), (x1, y1), line_id=usual_line_id, color="black")
            window.update()

        if i < FuzzyP[1]:
            x3 = curvesFuzzy[0][i]
            y3 = curvesFuzzy[1][i]
            canvas.coords(oval3, x3 - 5.0, y3 + 5.0, x3 + 5.0, y3 - 5.0)
            fuzzy_line_id = draw_sight_line(canvas, (x3, y3), (x1, y1), line_id=fuzzy_line_id, color="black")
            window.update()

        if not FuzzyFlag:
            FuzzyFlag = i == FuzzyP[1]
            if FuzzyFlag:
                distanceFuzz.config(text="0.00")
        if not UsualFlag:
            UsualFlag = i == UsualP[1]
            if UsualFlag:
                distanceUsual.config(text="0.00")

        if FuzzyFlag and UsualFlag:
            # distanceFuzz.config(text="0.00")
            # distanceUsual.config(text="0.00")
            break


def reset():
    canvas.delete("all")
    hitUsual.config(text='False', bg='red')
    hitFuzz.config(text='False', bg='red')
    distanceUsual.config(text='N/A')
    distanceFuzz.config(text='N/A')
    bearingUsual.config(text="0.00°")
    bearingFuzz.config(text="0.00°")
    Plane.clear()


def rocket():
    def btn_1(event):
        global prev_x, prev_y
        global point1, point2
        x = canvas.canvasx(event.x)
        y = canvas.canvasy(event.y)
        if prev_x:
            canvas.create_line(prev_x, prev_y, x, y, arrow=LAST)
            point1 = {"x": prev_x, "y": prev_y}
            point2 = {"x": x, "y": y}
            x = None
            y = None
        prev_x = x
        prev_y = y

    canvas.bind('<Button-1>', btn_1)


def plane():
    global Plane

    def btn_1(event):
        x = canvas.canvasx(event.x)
        y = canvas.canvasy(event.y)
        canvas.create_oval(x - 3.0, y + 3.0, x + 3.0, y - 3.0, fill="BLACK")

        point = {"x": x, "y": y}
        Plane.append(point)

    canvas.bind('<Button-1>', btn_1)


if __name__ == "__main__":
    window = Tk()
    window.title("Модель наведения ЛА на цель")
    window.geometry('1900x800')

    main_frame = Frame(window)
    main_frame.pack(side=BOTTOM, fill=X)
    btn_start = Button(main_frame, text="Start", command=start)
    btn_start.grid(row=0, column=0, padx=5, pady=5, sticky=N + S + W + E)

    btn_reset = Button(main_frame, text="Reset", command=reset)
    btn_reset.grid(row=0, column=1, padx=5, pady=5, sticky=N + S + W + E)

    # lbl_Methods = Label(main_frame, text="Fuzzy - Метод правого максимума; Defuzzy - Метод максимума-минимума")
    # lbl_Methods.grid(row=0, column=3, padx=5, pady=5)

    img1 = Image.open("img/plane.png")
    img_resized = img1.resize((50, 50))  # specify the desired width and height
    img_plane = ImageTk.PhotoImage(img_resized)
    btn_plane = Button(main_frame, text="Самолет", command=plane, image=img_plane)
    btn_plane.grid(row=1, column=0)

    img2 = Image.open("img/rocket.png")
    img_resized = img2.resize((50, 50))  # specify the desired width and height
    img_rocket = ImageTk.PhotoImage(img_resized)
    btn_rocket = Button(main_frame, text="Ракета", command=rocket, image=img_rocket)
    btn_rocket.grid(row=2, column=0)

    lbl_points = Label(main_frame, text="Дальность пути")
    lbl_points.grid(row=1, column=1, padx=5, pady=5)
    points = Entry(main_frame, width=10)
    points.insert(END, "800")
    points.grid(row=1, column=2, padx=5, pady=5)

    lbl_velocity = Label(main_frame, text="Скорость ракеты")
    lbl_velocity.grid(row=2, column=1, padx=5, pady=5)
    velocity = Entry(main_frame, width=10)
    velocity.insert(END, "7")
    velocity.grid(row=2, column=2, padx=5, pady=5)

    lbl_hitUsual = Label(main_frame, text="Метод пропорционального наведения")
    lbl_hitUsual.grid(row=1, column=3, padx=5, pady=5)
    lbl_color_hitUsual = Label(main_frame, bg='blue', width=5)
    lbl_color_hitUsual.grid(row=1, column=4, padx=5, pady=5)
    hitUsual = Label(main_frame, text="False")
    hitUsual.grid(row=1, column=5, padx=5, pady=5)

    lnl_hit = Label(main_frame, text="Попадание")
    lnl_hit.grid(row=0, column=5, padx=5, pady=5)
    lbl_distanceUsual = Label(main_frame, text="Расстояние до цели")
    lbl_distanceUsual.grid(row=0, column=6, padx=5, pady=5)
    distanceUsual = Label(main_frame, text="N/A")
    distanceUsual.grid(row=1, column=6, padx=5, pady=5)

    bearingUsualLabel = Label(main_frame, text="Пеленг")  # Add bearing label
    bearingUsualLabel.grid(row=0, column=7)
    bearingUsual = Label(main_frame, text="0.00°")
    bearingUsual.grid(row=1, column=7, padx=5, pady=5)

    lbl_hitFuzz = Label(main_frame, text="Нечёткое пропорциональное наведение")
    lbl_hitFuzz.grid(row=2, column=3, padx=5, pady=5)
    lbl_color_hitFuzz = Label(main_frame, bg='red', width=5)
    lbl_color_hitFuzz.grid(row=2, column=4, padx=5, pady=5)
    hitFuzz = Label(main_frame, text="False")
    hitFuzz.grid(row=2, column=5, padx=5, pady=5)

    distanceFuzz = Label(main_frame, text="N/A")
    distanceFuzz.grid(row=2, column=6, padx=5, pady=5)

    bearingFuzz = Label(main_frame, text="0.00°")
    bearingFuzz.grid(row=2, column=7, padx=5, pady=5)

    canvas = Canvas(window, relief=RAISED, borderwidth=1, bg='WHITE')
    canvas.pack(side=RIGHT, padx=5)
    canvas.pack(fill=BOTH, expand=1)

    canvas.bind('<ButtonPress-3>', lambda event: canvas.scan_mark(event.x, event.y))
    canvas.bind("<B3-Motion>", lambda event: canvas.scan_dragto(event.x, event.y, gain=1))

    st = Label(canvas, bg='WHITE')
    st.pack(side=TOP, expand=1)

    window.mainloop()
