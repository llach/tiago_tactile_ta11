def mean(l):
    o = 0
    for p in l:
        o += p
    return o/len(l)

### SENSOR 1 ###
# _b = 0.119775017855
# weights = [
#     45,
#     351,
#     176
# ]
# values = [
#     -0.209294331159,
#     -0.803804417738,
#     -0.462322927678
# ]
### SENSOR 1 ###

### SENSOR 2 ###
_b = 0.157910567142
weights = [
    45,
    351,
    176
]
values = [
    -0.248041967528,
    -0.842962215554,
    -0.500616231367
]
### SENSOR 2 ###

ms = []

print("Raw values:")
for w, v in zip(weights, values):
    print('{}: {}'.format(w, v))
    v += _b
print("")

for i in range(len(values)):
    values[i] = values[i] + _b

print("Bias-adjusted values:")
for w, v in zip(weights, values):
    print('{}: {}'.format(w, v))
    v += _b
print("")

print("ms:")
for w, v in zip(weights, values):
    _m = w/v
    print('{}/{} = {}'.format(w, v, _m))
    ms.append(_m)
print("")

print("m = {}".format(mean(ms)))