import euclid3
q = euclid3.Quaternion(1, 0.5, 0, 0)
euler = q.get_euler()
print(q)
print(euler[1])
print(type(euler[1]))
