// ======== LESSON 14: MOBILE BASES (shared core, sandbox 14, explainer) ========
(() => {
// Recorded from MuJoCo 3.15.0 by 14_mobile_bases.py (it writes 14_recorded_runs.json).
// Each path is the chassis pose [x (m, world), y (m, world), yaw (rad)] every 0.2 s.
const REC = {"B_eff":1.4314,"k_v":0.9776,"B_cal":1.3452,"runs":{"nominal":{"label":"nominal floor (mu 1.5)","open":{"err":0.0211,"t":12.86,"path":[[0.0,0.0,0.0],[0.058,0.0,0.0],[0.116,0.0,0.0],[0.175,0.0,0.0],[0.234,0.0,0.0],[0.293,-0.0,0.0],[0.352,0.0,0.0],[0.41,0.0,0.0],[0.469,0.0,0.0],[0.528,0.0,0.0],[0.587,-0.0,0.0],[0.645,0.0,0.0],[0.704,0.0,0.0],[0.763,0.0,0.0],[0.822,-0.0,0.0],[0.88,0.0,0.0],[0.939,0.0,0.0],[0.998,0.0,0.0],[1.057,0.0,0.0],[1.115,0.0,0.0],[1.174,0.0,0.0],[1.233,0.0,0.0],[1.292,0.0,0.0],[1.351,0.0,0.0],[1.409,0.0,0.0],[1.468,0.0,0.0],[1.473,0.0,0.0],[1.473,0.0,0.0],[1.473,0.0,0.033],[1.473,0.0,0.117],[1.473,0.0,0.209],[1.473,0.0,0.301],[1.473,0.0,0.39],[1.473,0.0,0.48],[1.473,0.0,0.569],[1.473,0.0,0.659],[1.473,0.0,0.747],[1.473,0.0,0.836],[1.473,0.0,0.925],[1.473,0.0,1.015],[1.473,0.0,1.104],[1.473,0.0,1.194],[1.473,0.0,1.282],[1.473,0.0,1.371],[1.473,0.0,1.46],[1.473,0.0,1.55],[1.473,0.0,1.555],[1.473,0.0,1.555],[1.473,0.022,1.555],[1.474,0.081,1.555],[1.475,0.139,1.555],[1.476,0.198,1.555],[1.477,0.257,1.555],[1.478,0.316,1.555],[1.478,0.374,1.555],[1.479,0.433,1.555],[1.48,0.492,1.555],[1.481,0.551,1.555],[1.482,0.609,1.555],[1.483,0.668,1.555],[1.484,0.727,1.555],[1.485,0.786,1.555],[1.486,0.844,1.555],[1.487,0.903,1.555],[1.488,0.962,1.555],[1.488,0.983,1.556],[1.488,0.983,1.556],[1.488,0.983,1.556],[1.488,0.983,1.556],[1.488,0.983,1.556]]},"closed":{"err":0.049,"t":21.99,"path":[[0.0,0.0,0.0],[0.066,0.0,0.0],[0.132,0.0,0.0],[0.194,0.0,0.0],[0.254,0.0,0.0],[0.311,0.0,0.0],[0.366,0.0,0.0],[0.418,0.0,0.0],[0.467,0.0,0.0],[0.514,0.0,0.0],[0.559,0.0,0.0],[0.602,0.0,0.0],[0.644,0.0,0.0],[0.683,0.0,0.0],[0.72,0.0,0.0],[0.756,0.0,0.0],[0.79,0.0,0.0],[0.822,0.0,0.0],[0.853,0.0,0.0],[0.883,0.0,0.0],[0.911,0.0,0.0],[0.938,0.0,0.0],[0.964,0.0,0.0],[0.988,0.0,0.0],[1.011,0.0,0.0],[1.034,0.0,0.0],[1.055,0.0,0.0],[1.075,0.0,0.0],[1.095,0.0,0.0],[1.113,0.0,0.0],[1.131,0.0,0.0],[1.149,0.0,0.0],[1.166,0.0,0.0],[1.184,0.0,0.0],[1.201,0.0,0.0],[1.219,0.0,0.0],[1.236,0.0,0.0],[1.254,0.0,0.0],[1.272,0.0,-0.0],[1.289,0.0,-0.0],[1.307,0.0,-0.0],[1.324,0.0,-0.0],[1.342,0.0,-0.0],[1.359,0.0,-0.0],[1.377,0.0,-0.0],[1.395,0.0,-0.0],[1.412,0.0,-0.0],[1.43,0.0,-0.0],[1.447,0.0,-0.0],[1.457,-0.001,0.078],[1.454,-0.003,0.177],[1.455,-0.005,0.306],[1.448,-0.008,0.402],[1.447,-0.003,0.498],[1.443,-0.003,0.585],[1.443,-0.002,0.668],[1.442,-0.004,0.747],[1.444,0.001,0.811],[1.448,0.005,0.874],[1.452,0.014,0.931],[1.459,0.026,0.981],[1.467,0.042,1.03],[1.477,0.055,1.078],[1.487,0.072,1.116],[1.494,0.09,1.151],[1.503,0.11,1.187],[1.51,0.13,1.218],[1.519,0.151,1.246],[1.526,0.172,1.271],[1.533,0.194,1.299],[1.539,0.216,1.321],[1.544,0.238,1.344],[1.549,0.261,1.368],[1.554,0.283,1.385],[1.558,0.305,1.404],[1.562,0.327,1.421],[1.565,0.349,1.437],[1.568,0.37,1.451],[1.571,0.391,1.468],[1.573,0.412,1.483],[1.574,0.432,1.501],[1.575,0.452,1.513],[1.576,0.471,1.524],[1.577,0.49,1.536],[1.578,0.508,1.547],[1.579,0.525,1.56],[1.579,0.543,1.571],[1.579,0.56,1.582],[1.579,0.578,1.592],[1.578,0.596,1.603],[1.578,0.614,1.613],[1.577,0.631,1.622],[1.576,0.649,1.633],[1.575,0.666,1.643],[1.573,0.684,1.652],[1.572,0.702,1.663],[1.571,0.719,1.672],[1.569,0.737,1.683],[1.567,0.755,1.692],[1.565,0.772,1.7],[1.563,0.789,1.709],[1.561,0.807,1.718],[1.558,0.824,1.728],[1.555,0.842,1.739],[1.553,0.859,1.75],[1.549,0.876,1.76],[1.546,0.893,1.773],[1.542,0.91,1.786],[1.539,0.927,1.8],[1.534,0.944,1.822],[1.53,0.961,1.841],[1.53,0.961,1.841],[1.53,0.961,1.841],[1.53,0.961,1.841],[1.53,0.961,1.841]]}},"slippery":{"label":"slippery floor (mu 0.3)","open":{"err":0.904,"t":12.86,"path":[[-0.0,0.0,-0.0],[0.044,-0.0,0.0],[0.103,0.0,-0.0],[0.161,-0.0,0.0],[0.22,-0.0,0.0],[0.279,-0.0,0.0],[0.337,-0.0,0.0],[0.396,-0.0,0.0],[0.455,-0.0,0.0],[0.514,-0.0,0.0],[0.572,-0.0,0.0],[0.631,-0.0,0.0],[0.69,-0.0,0.0],[0.749,0.0,0.0],[0.807,0.0,0.0],[0.866,0.0,0.0],[0.925,0.0,0.0],[0.983,0.0,0.0],[1.042,0.0,0.0],[1.101,0.0,0.0],[1.159,0.0,0.0],[1.218,0.0,0.0],[1.277,0.0,0.0],[1.336,0.0,0.0],[1.394,0.0,0.0],[1.453,0.0,0.0],[1.468,0.0,0.0],[1.468,0.0,0.0],[1.468,0.0,0.041],[1.467,0.0,0.181],[1.467,0.0,0.323],[1.468,0.0,0.466],[1.468,0.0,0.609],[1.467,-0.0,0.752],[1.467,-0.0,0.893],[1.467,-0.0,1.035],[1.467,-0.001,1.179],[1.467,-0.0,1.321],[1.467,0.0,1.463],[1.467,0.0,1.605],[1.467,-0.0,1.747],[1.467,-0.0,1.887],[1.467,0.0,2.032],[1.467,-0.0,2.175],[1.466,-0.001,2.316],[1.466,-0.001,2.458],[1.467,-0.001,2.486],[1.467,-0.001,2.486],[1.459,0.005,2.486],[1.414,0.04,2.486],[1.367,0.076,2.486],[1.321,0.111,2.486],[1.274,0.147,2.486],[1.228,0.183,2.486],[1.181,0.219,2.486],[1.134,0.255,2.486],[1.088,0.291,2.486],[1.041,0.326,2.486],[0.995,0.362,2.486],[0.948,0.398,2.486],[0.902,0.434,2.486],[0.855,0.47,2.486],[0.809,0.506,2.486],[0.762,0.541,2.486],[0.716,0.577,2.486],[0.691,0.596,2.486],[0.691,0.596,2.486],[0.691,0.596,2.486],[0.691,0.596,2.486],[0.691,0.596,2.486]]},"closed":{"err":0.0485,"t":20.54,"path":[[-0.0,0.0,-0.0],[0.048,-0.0,0.0],[0.115,-0.0,0.0],[0.178,-0.0,-0.0],[0.239,-0.0,-0.0],[0.297,-0.0,-0.0],[0.352,-0.0,-0.0],[0.404,-0.0,-0.0],[0.454,-0.0,-0.0],[0.502,-0.0,-0.0],[0.548,-0.0,-0.0],[0.592,-0.0,-0.0],[0.633,-0.0,-0.0],[0.673,-0.0,-0.0],[0.711,-0.0,-0.0],[0.747,-0.0,-0.0],[0.781,-0.0,-0.0],[0.814,-0.0,-0.0],[0.846,-0.0,-0.0],[0.876,-0.0,0.0],[0.904,-0.0,0.0],[0.931,-0.0,0.0],[0.957,-0.0,0.0],[0.982,-0.0,0.0],[1.006,-0.0,0.0],[1.028,-0.0,0.0],[1.05,-0.0,0.0],[1.071,-0.0,0.0],[1.09,-0.0,0.0],[1.109,-0.0,0.0],[1.127,-0.0,0.0],[1.144,-0.0,0.0],[1.162,-0.0,0.0],[1.18,-0.0,0.0],[1.197,-0.0,0.0],[1.215,-0.0,0.0],[1.232,-0.0,0.0],[1.25,-0.0,0.0],[1.267,-0.0,0.0],[1.285,-0.0,0.0],[1.303,-0.0,0.0],[1.32,-0.0,0.0],[1.338,-0.0,0.0],[1.355,-0.0,0.0],[1.373,-0.0,0.0],[1.39,-0.0,0.0],[1.408,-0.0,0.0],[1.426,-0.0,0.0],[1.443,-0.0,0.0],[1.459,-0.0,0.058],[1.467,-0.0,0.22],[1.47,-0.001,0.387],[1.471,-0.002,0.532],[1.471,-0.002,0.66],[1.472,-0.002,0.771],[1.473,0.001,0.866],[1.477,0.007,0.95],[1.484,0.018,1.024],[1.492,0.032,1.091],[1.501,0.049,1.15],[1.509,0.069,1.203],[1.518,0.091,1.25],[1.525,0.115,1.293],[1.532,0.14,1.331],[1.538,0.166,1.366],[1.543,0.192,1.398],[1.548,0.219,1.426],[1.552,0.246,1.452],[1.555,0.273,1.475],[1.557,0.299,1.495],[1.559,0.325,1.513],[1.56,0.351,1.53],[1.561,0.376,1.545],[1.561,0.4,1.559],[1.562,0.424,1.573],[1.561,0.447,1.585],[1.561,0.47,1.597],[1.56,0.492,1.607],[1.559,0.513,1.617],[1.558,0.533,1.626],[1.557,0.553,1.634],[1.556,0.572,1.642],[1.554,0.59,1.648],[1.553,0.608,1.655],[1.552,0.625,1.661],[1.55,0.643,1.666],[1.548,0.66,1.671],[1.546,0.678,1.675],[1.545,0.695,1.68],[1.543,0.713,1.684],[1.541,0.73,1.687],[1.538,0.748,1.691],[1.536,0.765,1.694],[1.534,0.782,1.698],[1.532,0.8,1.701],[1.53,0.817,1.704],[1.527,0.835,1.707],[1.525,0.852,1.71],[1.522,0.87,1.713],[1.52,0.887,1.716],[1.517,0.904,1.719],[1.515,0.922,1.723],[1.512,0.939,1.727],[1.51,0.953,1.731],[1.51,0.953,1.731],[1.51,0.953,1.731],[1.51,0.953,1.731],[1.51,0.953,1.731]]}},"heavy":{"label":"payload +5 kg","open":{"err":0.1038,"t":12.86,"path":[[0.0,-0.0,-0.0],[0.058,0.0,0.0],[0.116,0.0,0.0],[0.175,0.0,0.0],[0.234,0.0,0.0],[0.292,0.0,0.0],[0.351,0.0,0.0],[0.41,0.0,0.0],[0.469,0.0,0.0],[0.527,0.0,0.0],[0.586,0.0,0.0],[0.645,0.0,0.0],[0.703,0.0,0.0],[0.762,0.0,0.0],[0.821,0.0,0.0],[0.879,0.0,0.0],[0.938,0.0,0.0],[0.997,0.0,0.0],[1.055,0.0,0.0],[1.114,0.0,0.0],[1.173,0.0,0.0],[1.231,0.0,0.0],[1.29,0.0,0.0],[1.349,0.0,0.0],[1.408,0.0,0.0],[1.466,0.0,0.0],[1.469,0.0,0.0],[1.469,0.0,0.0],[1.469,0.0,0.03],[1.469,0.0,0.111],[1.469,0.0,0.192],[1.469,0.0,0.275],[1.469,0.0,0.358],[1.469,0.0,0.437],[1.469,0.0,0.522],[1.469,0.0,0.604],[1.469,0.0,0.687],[1.469,0.0,0.767],[1.469,0.0,0.851],[1.469,0.0,0.932],[1.469,0.0,1.012],[1.469,0.0,1.097],[1.469,0.0,1.18],[1.469,0.0,1.261],[1.469,0.0,1.342],[1.469,0.0,1.426],[1.469,0.0,1.437],[1.469,0.0,1.437],[1.472,0.022,1.437],[1.48,0.08,1.437],[1.488,0.138,1.437],[1.496,0.196,1.437],[1.503,0.254,1.437],[1.511,0.313,1.437],[1.519,0.371,1.437],[1.527,0.429,1.437],[1.535,0.487,1.437],[1.543,0.545,1.437],[1.55,0.603,1.437],[1.558,0.662,1.437],[1.566,0.72,1.437],[1.574,0.778,1.437],[1.582,0.836,1.437],[1.589,0.894,1.437],[1.597,0.952,1.437],[1.6,0.972,1.437],[1.6,0.972,1.437],[1.6,0.972,1.437],[1.6,0.972,1.437],[1.6,0.972,1.437]]},"closed":{"err":0.0502,"t":24.12,"path":[[0.0,-0.0,-0.0],[0.066,0.0,0.0],[0.132,0.0,0.0],[0.194,0.0,0.0],[0.254,0.0,0.0],[0.311,0.0,0.0],[0.365,0.0,0.0],[0.417,0.0,0.0],[0.467,0.0,0.0],[0.514,0.0,0.0],[0.559,0.0,0.0],[0.602,0.0,0.0],[0.643,0.0,0.0],[0.682,0.0,0.0],[0.72,0.0,0.0],[0.755,0.0,0.0],[0.789,0.0,0.0],[0.822,0.0,0.0],[0.853,0.0,0.0],[0.882,0.0,0.0],[0.91,0.0,0.0],[0.937,0.0,0.0],[0.963,0.0,0.0],[0.987,0.0,0.0],[1.011,0.0,0.0],[1.033,0.0,0.0],[1.054,0.0,0.0],[1.075,0.0,0.0],[1.094,0.0,0.0],[1.113,0.0,0.0],[1.13,0.0,0.0],[1.148,0.0,0.0],[1.165,0.0,0.0],[1.183,0.0,0.0],[1.2,0.0,0.0],[1.218,0.0,0.0],[1.235,0.0,0.0],[1.253,0.0,0.0],[1.27,0.0,0.0],[1.288,0.0,0.0],[1.306,0.0,-0.0],[1.323,0.0,-0.0],[1.341,0.0,-0.0],[1.358,0.0,-0.0],[1.376,0.0,-0.0],[1.393,0.0,-0.0],[1.411,0.0,-0.0],[1.428,0.0,-0.0],[1.446,0.0,-0.0],[1.455,-0.002,0.06],[1.458,0.0,0.144],[1.455,0.0,0.235],[1.458,-0.002,0.328],[1.462,0.0,0.43],[1.463,-0.0,0.513],[1.466,0.003,0.594],[1.461,-0.0,0.652],[1.459,-0.0,0.723],[1.46,0.001,0.797],[1.462,0.004,0.846],[1.467,0.009,0.894],[1.473,0.015,0.942],[1.479,0.025,0.987],[1.486,0.039,1.026],[1.494,0.053,1.055],[1.501,0.068,1.091],[1.51,0.083,1.121],[1.518,0.1,1.156],[1.526,0.118,1.187],[1.534,0.137,1.215],[1.542,0.156,1.241],[1.548,0.176,1.266],[1.555,0.197,1.29],[1.56,0.218,1.312],[1.565,0.239,1.334],[1.571,0.26,1.353],[1.575,0.28,1.373],[1.579,0.301,1.39],[1.583,0.321,1.407],[1.586,0.342,1.423],[1.589,0.361,1.44],[1.592,0.381,1.452],[1.594,0.4,1.465],[1.596,0.42,1.478],[1.597,0.438,1.491],[1.599,0.457,1.505],[1.6,0.475,1.517],[1.601,0.493,1.528],[1.602,0.511,1.543],[1.602,0.529,1.553],[1.602,0.547,1.565],[1.602,0.564,1.578],[1.602,0.582,1.59],[1.601,0.599,1.601],[1.601,0.617,1.612],[1.6,0.634,1.623],[1.599,0.652,1.635],[1.598,0.669,1.646],[1.596,0.686,1.657],[1.594,0.704,1.668],[1.593,0.722,1.68],[1.592,0.74,1.692],[1.59,0.759,1.704],[1.588,0.776,1.717],[1.586,0.794,1.731],[1.583,0.811,1.745],[1.579,0.828,1.758],[1.575,0.845,1.773],[1.572,0.862,1.787],[1.568,0.88,1.803],[1.564,0.897,1.82],[1.559,0.914,1.841],[1.554,0.931,1.863],[1.553,0.941,1.891],[1.552,0.944,1.92],[1.55,0.95,1.95],[1.547,0.955,1.978],[1.546,0.959,2.012],[1.543,0.964,2.041],[1.543,0.966,2.068],[1.542,0.968,2.094],[1.54,0.97,2.111],[1.54,0.97,2.111],[1.54,0.97,2.111],[1.54,0.97,2.111],[1.54,0.97,2.111]]}},"slowright":{"label":"right motors 10% slow","open":{"err":0.1419,"t":12.86,"path":[[0.0,0.0,0.0],[0.055,-0.0,-0.002],[0.111,-0.0,-0.003],[0.167,-0.001,-0.005],[0.222,-0.001,-0.006],[0.278,-0.001,-0.007],[0.334,-0.002,-0.009],[0.39,-0.002,-0.011],[0.446,-0.003,-0.012],[0.502,-0.004,-0.014],[0.557,-0.004,-0.015],[0.613,-0.005,-0.017],[0.669,-0.006,-0.018],[0.725,-0.007,-0.019],[0.781,-0.009,-0.021],[0.836,-0.01,-0.022],[0.892,-0.011,-0.024],[0.948,-0.012,-0.026],[1.004,-0.014,-0.027],[1.06,-0.015,-0.029],[1.115,-0.017,-0.03],[1.171,-0.019,-0.032],[1.227,-0.021,-0.033],[1.282,-0.022,-0.035],[1.338,-0.024,-0.036],[1.394,-0.027,-0.037],[1.398,-0.027,-0.037],[1.398,-0.027,-0.037],[1.397,-0.027,-0.004],[1.391,-0.027,0.071],[1.39,-0.027,0.142],[1.388,-0.03,0.233],[1.386,-0.031,0.316],[1.385,-0.031,0.394],[1.384,-0.031,0.481],[1.385,-0.031,0.566],[1.383,-0.032,0.648],[1.38,-0.033,0.742],[1.378,-0.037,0.838],[1.373,-0.041,0.911],[1.371,-0.046,0.995],[1.369,-0.047,1.072],[1.368,-0.049,1.154],[1.368,-0.055,1.252],[1.366,-0.059,1.34],[1.364,-0.062,1.431],[1.364,-0.065,1.446],[1.364,-0.065,1.446],[1.367,-0.044,1.445],[1.374,0.011,1.443],[1.381,0.066,1.442],[1.388,0.122,1.44],[1.396,0.177,1.439],[1.403,0.232,1.438],[1.411,0.288,1.436],[1.418,0.343,1.434],[1.426,0.399,1.433],[1.433,0.454,1.431],[1.441,0.509,1.43],[1.449,0.564,1.428],[1.457,0.62,1.427],[1.465,0.675,1.425],[1.473,0.73,1.424],[1.482,0.785,1.422],[1.49,0.841,1.421],[1.493,0.858,1.42],[1.493,0.858,1.42],[1.493,0.858,1.42],[1.493,0.858,1.42],[1.493,0.858,1.42]]},"closed":{"err":0.0482,"t":24.3,"path":[[0.0,0.0,0.0],[0.063,-0.0,-0.002],[0.125,-0.0,-0.004],[0.185,-0.001,-0.005],[0.242,-0.001,-0.006],[0.296,-0.001,-0.007],[0.348,-0.002,-0.008],[0.397,-0.002,-0.009],[0.444,-0.003,-0.01],[0.49,-0.003,-0.011],[0.533,-0.004,-0.011],[0.574,-0.004,-0.012],[0.613,-0.004,-0.012],[0.651,-0.005,-0.012],[0.687,-0.005,-0.012],[0.722,-0.006,-0.013],[0.754,-0.006,-0.013],[0.786,-0.007,-0.013],[0.816,-0.007,-0.013],[0.845,-0.007,-0.013],[0.872,-0.008,-0.013],[0.899,-0.008,-0.013],[0.924,-0.008,-0.013],[0.948,-0.009,-0.012],[0.971,-0.009,-0.012],[0.993,-0.009,-0.012],[1.014,-0.01,-0.012],[1.034,-0.01,-0.012],[1.054,-0.01,-0.011],[1.072,-0.01,-0.011],[1.09,-0.01,-0.011],[1.107,-0.011,-0.011],[1.123,-0.011,-0.01],[1.14,-0.011,-0.01],[1.156,-0.011,-0.01],[1.173,-0.011,-0.009],[1.189,-0.011,-0.009],[1.206,-0.012,-0.008],[1.222,-0.012,-0.008],[1.239,-0.012,-0.007],[1.256,-0.012,-0.007],[1.272,-0.012,-0.006],[1.289,-0.012,-0.005],[1.305,-0.012,-0.004],[1.321,-0.012,-0.003],[1.338,-0.012,-0.002],[1.354,-0.012,0.001],[1.371,-0.012,0.003],[1.387,-0.012,0.006],[1.403,-0.012,0.009],[1.42,-0.012,0.014],[1.436,-0.012,0.021],[1.452,-0.011,0.031],[1.455,-0.013,0.124],[1.451,-0.01,0.24],[1.446,-0.015,0.355],[1.443,-0.016,0.459],[1.438,-0.02,0.549],[1.432,-0.024,0.634],[1.431,-0.027,0.709],[1.431,-0.029,0.771],[1.433,-0.027,0.839],[1.435,-0.022,0.894],[1.442,-0.015,0.944],[1.45,-0.006,0.99],[1.458,0.005,1.031],[1.466,0.019,1.068],[1.474,0.033,1.102],[1.483,0.049,1.135],[1.491,0.067,1.168],[1.498,0.085,1.194],[1.506,0.104,1.22],[1.513,0.124,1.245],[1.52,0.145,1.266],[1.526,0.166,1.287],[1.533,0.186,1.304],[1.538,0.207,1.325],[1.543,0.227,1.345],[1.548,0.248,1.365],[1.552,0.269,1.381],[1.557,0.289,1.398],[1.56,0.31,1.414],[1.563,0.33,1.429],[1.565,0.35,1.444],[1.567,0.37,1.459],[1.569,0.39,1.471],[1.571,0.409,1.481],[1.572,0.428,1.492],[1.574,0.446,1.503],[1.575,0.464,1.516],[1.576,0.482,1.529],[1.577,0.499,1.54],[1.578,0.515,1.551],[1.578,0.532,1.561],[1.578,0.548,1.57],[1.578,0.564,1.578],[1.577,0.58,1.586],[1.577,0.596,1.596],[1.577,0.612,1.606],[1.577,0.628,1.614],[1.576,0.645,1.621],[1.575,0.661,1.628],[1.574,0.677,1.637],[1.573,0.693,1.645],[1.572,0.709,1.653],[1.57,0.725,1.66],[1.568,0.741,1.669],[1.567,0.757,1.679],[1.565,0.773,1.686],[1.563,0.789,1.695],[1.561,0.805,1.704],[1.558,0.821,1.714],[1.556,0.836,1.722],[1.554,0.852,1.73],[1.551,0.868,1.74],[1.549,0.884,1.752],[1.546,0.9,1.765],[1.543,0.916,1.781],[1.54,0.931,1.801],[1.536,0.947,1.823],[1.533,0.959,1.851],[1.534,0.961,1.877],[1.532,0.964,1.895],[1.532,0.964,1.895],[1.532,0.964,1.895],[1.532,0.964,1.895],[1.532,0.964,1.895]]}}}};

// SIM-CORE-BEGIN
// A kinematic unicycle, NOT MuJoCo physics: no contacts, no slip, no wheel inertia.
// It moves the base with the skid-steer formulas, calibrated on MuJoCo by 14_mobile_bases.py:
//   v = KV * r * (wL + wR) / 2      (m/s)    KV = 0.9776 (wheels run a little under their command)
//   w = r * (wR - wL) / B           (rad/s)  B = 1.4314 m, the effective track width (geometric 0.34 m)
// wL, wR are the wheel COMMANDS (rad/s). The right wheels get (1 + bias) times their command.
// Integration: th += w dt, then x += v cos(th) dt, y += v sin(th) dt, dt = 0.01 s (same as the script).
const K14 = { R: 0.06, B: REC.B_eff, KV: REC.k_v, DT: 0.01, KPH: 6, KDH: 2, KPS: 4, MAXW: 6, ARRIVE: 0.05 };
function k14Wrap(a) { const p = 2 * Math.PI; let m = (a + Math.PI) % p; if (m < 0) m += p; return m - Math.PI; }
function k14Clip(v, lo, hi) { return Math.min(hi, Math.max(lo, v)); }
function k14Step(s, cl, cr, bias) {
  const crb = cr * (1 + bias);
  const v = K14.KV * K14.R * (cl + crb) / 2, w = K14.R * (crb - cl) / K14.B;
  s.th += w * K14.DT; s.x += v * Math.cos(s.th) * K14.DT; s.y += v * Math.sin(s.th) * K14.DT; s.t += K14.DT;
  s.wz = w; s.cl = cl; s.cr = crb;
}
const k14Snap = s => [s.t, s.x, s.y, s.th, s.wz, s.cl, s.cr];
// Closed loop: the Capstone 2 controller (heading PD with gyro damping, speed from distance, gated by heading error).
function k14Closed(wps, bias, tmax = 60) {
  const s = { x: 0, y: 0, th: 0, t: 0, wz: 0, cl: 0, cr: 0 }, path = [k14Snap(s)];
  let k = 0;
  while (s.t < tmax && k < wps.length) {
    const dx = wps[k][0] - s.x, dy = wps[k][1] - s.y, dist = Math.hypot(dx, dy);
    if (dist < K14.ARRIVE) { k++; continue; }
    const he = k14Wrap(Math.atan2(dy, dx) - s.th);
    const turn = k14Clip(K14.KPH * he - K14.KDH * s.wz, -K14.MAXW, K14.MAXW);
    const al = Math.max(0, 1 - Math.abs(he) / 0.8), creep = Math.abs(he) < 0.4 ? 1.5 : 0;
    const sp = k14Clip(Math.max(K14.KPS * dist * al, creep), 0, K14.MAXW);
    k14Step(s, k14Clip(sp - turn, -K14.MAXW, K14.MAXW), k14Clip(sp + turn, -K14.MAXW, K14.MAXW), bias);
    path.push(k14Snap(s));
  }
  const last = wps[wps.length - 1];
  return { path, arrived: k >= wps.length, t: s.t, err: Math.hypot(s.x - last[0], s.y - last[1]) };
}
// Open loop: plan every command and duration in advance (turn in place at -5/+5, drive at 5/5, 0.5 s stops),
// using the same formulas as the world, then play the plan without looking.
function k14Open(wps, bias) {
  const s = { x: 0, y: 0, th: 0, t: 0, wz: 0, cl: 0, cr: 0 }, path = [k14Snap(s)];
  let bx = 0, by = 0, bth = 0;
  const run = (cl, cr, T) => { const n = Math.round(T / K14.DT); for (let i = 0; i < n; i++) { k14Step(s, cl, cr, bias); path.push(k14Snap(s)); } };
  wps.forEach(([wx, wy]) => {
    const d = k14Wrap(Math.atan2(wy - by, wx - bx) - bth);
    if (Math.abs(d) > 1e-9) { run(d > 0 ? -5 : 5, d > 0 ? 5 : -5, Math.abs(d) / (K14.R * 10 / K14.B)); run(0, 0, 0.5); }
    run(5, 5, Math.hypot(wx - bx, wy - by) / (K14.KV * K14.R * 5)); run(0, 0, 0.5);
    bth += d; bx = wx; by = wy;
  });
  const last = wps[wps.length - 1];
  return { path, arrived: true, t: s.t - 0.5, err: Math.hypot(s.x - last[0], s.y - last[1]) };
}
// SIM-CORE-END

const $ = id => document.getElementById(id);
const ROUTE = [[1.5, 0], [1.5, 1.0]];
const deg = r => r * 180 / Math.PI;
const CASES = ['nominal', 'slippery', 'heavy', 'slowright'];
const CASE_SHORT = { nominal: 'normal floor', slippery: 'slippery floor (mu 0.3)', heavy: 'payload +5 kg', slowright: 'right motors 10% slow' };

// ======== SANDBOX 14 ========
(() => {
  const NS = 'http://www.w3.org/2000/svg';
  const X = x => 20 + (x + 0.6) * 96, Y = y => 400 - (y + 1.0) * 96;
  const IX = px => (px - 20) / 96 - 0.6, IY = py => (400 - py) / 96 - 1.0;
  const svg = $('s14Svg'), grid = $('s14Grid');
  // 0.5 m grid with labels every metre
  for (let gx = -0.5; gx <= 3.0001; gx += 0.5) {
    const l = document.createElementNS(NS, 'line'); l.setAttribute('x1', X(gx)); l.setAttribute('x2', X(gx)); l.setAttribute('y1', Y(2.7)); l.setAttribute('y2', Y(-1.0));
    l.setAttribute('stroke', 'var(--border)'); l.setAttribute('stroke-width', Math.abs(gx % 1) < 1e-9 ? 1 : 0.5); grid.appendChild(l);
    if (Math.abs(gx % 1) < 1e-9) { const t = document.createElementNS(NS, 'text'); t.setAttribute('x', X(gx) + 3); t.setAttribute('y', Y(-1.0) - 4); t.setAttribute('fill', 'var(--text-dim)'); t.setAttribute('font-size', '10'); t.setAttribute('font-family', 'var(--mono)'); t.textContent = gx.toFixed(0) + ' m'; grid.appendChild(t); }
  }
  for (let gy = -1.0; gy <= 2.7001; gy += 0.5) {
    const l = document.createElementNS(NS, 'line'); l.setAttribute('y1', Y(gy)); l.setAttribute('y2', Y(gy)); l.setAttribute('x1', X(-0.6)); l.setAttribute('x2', X(3.1));
    l.setAttribute('stroke', 'var(--border)'); l.setAttribute('stroke-width', Math.abs(gy % 1) < 1e-9 ? 1 : 0.5); grid.appendChild(l);
    if (Math.abs(gy % 1) < 1e-9 && gy > -1) { const t = document.createElementNS(NS, 'text'); t.setAttribute('x', X(-0.6) + 3); t.setAttribute('y', Y(gy) - 3); t.setAttribute('fill', 'var(--text-dim)'); t.setAttribute('font-size', '10'); t.setAttribute('font-family', 'var(--mono)'); t.textContent = gy.toFixed(0) + ' m'; grid.appendChild(t); }
  }
  $('s14WorldX').setAttribute('x1', X(0)); $('s14WorldX').setAttribute('y1', Y(0)); $('s14WorldX').setAttribute('x2', X(0.5)); $('s14WorldX').setAttribute('y2', Y(0));
  $('s14WorldY').setAttribute('x1', X(0)); $('s14WorldY').setAttribute('y1', Y(0)); $('s14WorldY').setAttribute('x2', X(0)); $('s14WorldY').setAttribute('y2', Y(0.5));
  $('s14Wall').setAttribute('x', X(2.95)); $('s14Wall').setAttribute('y', Y(2.7)); $('s14Wall').setAttribute('width', 0.1 * 96); $('s14Wall').setAttribute('height', 3.7 * 96);

  let wps = ROUTE.map(p => p.slice()), res = null, token = 0, overlay = null;
  const state = { t: 0, x: 0, y: 0, th: 0, wz: 0, cl: 0, cr: 0 };
  const bias = () => +$('s14Bias').value / 100;

  function range(x, y, th) {
    // rangefinder on the chassis front (0.26 m ahead), ray along the heading, wall face at x = 2.95, |y| <= 3
    const sx = x + 0.26 * Math.cos(th), sy = y + 0.26 * Math.sin(th), c = Math.cos(th);
    if (c <= 1e-9) return -1;
    const d = (2.95 - sx) / c, hy = sy + d * Math.sin(th);
    return (d >= 0 && Math.abs(hy) <= 3) ? d : -1;
  }
  function drawWps() {
    const g = $('s14Wps'); g.innerHTML = '';
    wps.forEach(([wx, wy], i) => {
      const c = document.createElementNS(NS, 'circle'); c.setAttribute('cx', X(wx)); c.setAttribute('cy', Y(wy)); c.setAttribute('r', 0.05 * 96); c.setAttribute('fill', 'none'); c.setAttribute('stroke', 'var(--yellow)'); c.setAttribute('stroke-width', 2); g.appendChild(c);
      const t = document.createElementNS(NS, 'text'); t.setAttribute('x', X(wx) + 8); t.setAttribute('y', Y(wy) - 8); t.setAttribute('fill', 'var(--yellow)'); t.setAttribute('font-size', '11'); t.setAttribute('font-family', 'var(--mono)'); t.textContent = (wps.length > 1 ? 'W' + (i + 1) + ' ' : 'goal ') + '(' + wx.toFixed(2) + ', ' + wy.toFixed(2) + ')'; g.appendChild(t);
    });
  }
  function drawOverlay() {
    const o = $('s14RecOpen'), c = $('s14RecClosed');
    if (!overlay) { o.setAttribute('points', ''); c.setAttribute('points', ''); $('s14RecNote').textContent = ''; return; }
    const r = REC.runs[overlay];
    o.setAttribute('points', r.open.path.map(p => X(p[0]).toFixed(1) + ',' + Y(p[1]).toFixed(1)).join(' '));
    c.setAttribute('points', r.closed.path.map(p => X(p[0]).toFixed(1) + ',' + Y(p[1]).toFixed(1)).join(' '));
    $('s14RecNote').textContent = 'dashed: MuJoCo, ' + CASE_SHORT[overlay];
  }
  function show(k, mode) {
    if (res) { const p = res.path[Math.min(k, res.path.length - 1)]; [state.t, state.x, state.y, state.th, state.wz, state.cl, state.cr] = p; }
    $('s14Robot').setAttribute('transform', `translate(${X(state.x).toFixed(2)} ${Y(state.y).toFixed(2)}) rotate(${(-deg(state.th)).toFixed(2)})`);
    if (res) $('s14Trail').setAttribute('points', res.path.slice(0, k + 1).filter((_, i) => i % 5 === 0 || i === k).map(p => X(p[1]).toFixed(1) + ',' + Y(p[2]).toFixed(1)).join(' '));
    else $('s14Trail').setAttribute('points', '');
    const last = wps[wps.length - 1], dist = Math.hypot(last[0] - state.x, last[1] - state.y), rf = range(state.x, state.y, state.th);
    $('s14X').textContent = state.x.toFixed(3); $('s14Y').textContent = state.y.toFixed(3);
    $('s14Yaw').textContent = deg(k14Wrap(state.th)).toFixed(1); $('s14Wz').textContent = state.wz.toFixed(3);
    $('s14Cmd').textContent = state.cl.toFixed(2) + ' / ' + state.cr.toFixed(2);
    $('s14Dist').textContent = dist.toFixed(3); $('s14Range').textContent = rf < 0 ? '-1 (no hit)' : rf.toFixed(3);
    $('s14Clock').textContent = 't = ' + state.t.toFixed(2) + ' s' + (mode ? '   ' + mode : '');
  }
  function finish(mode) {
    $('s14Status').textContent = mode + ': ' + (res.arrived ? 'arrived' : 'gave up after 60 s') + ', final distance to the last goal ' + res.err.toFixed(3) + ' m, time ' + res.t.toFixed(2) + ' s (kinematic model, bias ' + Math.round(bias() * 100) + '%)';
  }
  function play(kind) {
    const my = ++token;
    res = kind === 'open' ? k14Open(wps, bias()) : k14Closed(wps, bias());
    { const e = res.path[res.path.length - 1]; res.path.push([e[0], e[1], e[2], e[3], 0, 0, 0]); }   // wheels stopped at the end
    const mode = kind === 'open' ? 'open loop (eyes closed)' : 'closed loop (eyes open)';
    $('s14Status').textContent = mode + ': running at 4x speed...';
    const N = res.path.length;
    if (RM) { show(N - 1, mode); finish(mode); return; }
    const t0 = performance.now();
    (function frame(now) {
      if (my !== token) return;
      const k = Math.min(N - 1, Math.max(0, Math.floor((now - t0) / 1000 * 4 / K14.DT)));
      show(k, mode);
      if (k < N - 1) requestAnimationFrame(frame); else finish(mode);
    })(t0);
  }
  function reset() { token++; res = null; Object.assign(state, { t: 0, x: 0, y: 0, th: 0, wz: 0, cl: 0, cr: 0 }); $('s14Status').textContent = 'Pick a goal, then press Open loop or Closed loop.'; show(0, ''); }
  function setGoal(x, y) {
    x = Math.round(k14Clip(x, -0.5, 2.8) * 20) / 20; y = Math.round(k14Clip(y, -0.9, 2.6) * 20) / 20;
    wps = [[x, y]]; overlay = null; drawOverlay();
    $('s14Gx').value = x; $('s14Gy').value = y; labels(); drawWps(); reset();
  }
  function labels() { $('s14GxVal').textContent = (+$('s14Gx').value).toFixed(2); $('s14GyVal').textContent = (+$('s14Gy').value).toFixed(2); $('s14BiasVal').textContent = (+$('s14Bias').value).toFixed(0); }
  svg.addEventListener('click', e => {
    const pt = svg.createSVGPoint(); pt.x = e.clientX; pt.y = e.clientY;
    const m = svg.getScreenCTM(); if (!m) return; const p = pt.matrixTransform(m.inverse());
    if (p.y > 400 || p.y < Y(2.7)) return;
    setGoal(IX(p.x), IY(p.y));
  });
  $('s14Gx').addEventListener('input', () => setGoal(+$('s14Gx').value, +$('s14Gy').value));
  $('s14Gy').addEventListener('input', () => setGoal(+$('s14Gx').value, +$('s14Gy').value));
  $('s14Bias').addEventListener('input', () => { labels(); reset(); });
  $('s14Open').addEventListener('click', () => play('open'));
  $('s14Closed').addEventListener('click', () => play('closed'));
  $('s14Reset').addEventListener('click', reset);
  $('s14Route').addEventListener('click', () => { wps = ROUTE.map(p => p.slice()); drawWps(); reset(); });
  document.querySelectorAll('#sandbox14 [data-case]').forEach(b => b.addEventListener('click', () => {
    overlay = b.dataset.case; wps = ROUTE.map(p => p.slice()); drawWps(); drawOverlay(); reset();
  }));
  // recorded MuJoCo results, written from REC (the script's numbers)
  $('s14RecTable').innerHTML = CASES.map(c => {
    const r = REC.runs[c];
    return `<span class="rk">${CASE_SHORT[c]}:</span> <span class="rk">open</span> <span class="rv-o">${r.open.err.toFixed(3)} m, ${r.open.t.toFixed(2)} s</span> <span class="rk">closed</span> <span class="rv">${r.closed.err.toFixed(3)} m, ${r.closed.t.toFixed(2)} s</span>`;
  }).join('<br>');
  labels(); drawWps(); reset();
})();

// ======== LESSON 14 EXPLAINER ========
(() => {
  // Only redraw this lesson's canvases while it is on screen (hidden canvases have zero size).
  const addEventListener = (type, fn, opts) => window.addEventListener(type, e => { if (document.getElementById('lesson14').classList.contains('active')) fn(e); }, opts);

  // Top-down floor view. v = {x0, x1, y0, y1} in metres (world). Returns world -> canvas mappers.
  function floor(c, w, h, v, step = 0.5) {
    const s = Math.max(1, Math.min((w - 16) / (v.x1 - v.x0), (h - 16) / (v.y1 - v.y0)));   // hidden canvas: w = 0
    const ox = (w - s * (v.x1 - v.x0)) / 2, oy = (h - s * (v.y1 - v.y0)) / 2;
    const P = (x, y) => [ox + (x - v.x0) * s, h - oy - (y - v.y0) * s];
    c.clearRect(0, 0, w, h);
    c.lineWidth = 1;
    for (let x = Math.ceil(v.x0 / step) * step; x <= v.x1 + 1e-9; x += step) {
      const on = Math.abs(x - Math.round(x)) < 1e-9; c.strokeStyle = css('--grid'); c.globalAlpha = on ? 1 : 0.5;
      c.beginPath(); c.moveTo(...P(x, v.y0)); c.lineTo(...P(x, v.y1)); c.stroke(); c.globalAlpha = 1;
      if (on) text(c, x.toFixed(0) + ' m', P(x, v.y0)[0] + 3, P(x, v.y0)[1] - 8, css('--dim'), 9);
    }
    for (let y = Math.ceil(v.y0 / step) * step; y <= v.y1 + 1e-9; y += step) {
      const on = Math.abs(y - Math.round(y)) < 1e-9; c.strokeStyle = css('--grid'); c.globalAlpha = on ? 1 : 0.5;
      c.beginPath(); c.moveTo(...P(v.x0, y)); c.lineTo(...P(v.x1, y)); c.stroke(); c.globalAlpha = 1;
      if (on && y > v.y0 + 1e-9) text(c, y.toFixed(0) + ' m', P(v.x0, y)[0] + 3, P(v.x0, y)[1] - 7, css('--dim'), 9);
    }
    const o = P(0, 0); arrow(c, o[0], o[1], P(0.4, 0)[0], o[1], css('--x'), 2, 7); arrow(c, o[0], o[1], o[0], P(0, 0.4)[1], css('--y'), 2, 7);
    text(c, 'x', P(0.45, 0)[0], o[1], css('--x'), 11); text(c, 'y', o[0] + 6, P(0, 0.46)[1], css('--y'), 11);
    return { P, s };
  }
  // The base seen from above: 0.5 x 0.3 m chassis, four wheels, heading arrow (body +x).
  function robot(c, P, s, x, y, th, col, alpha = 1) {
    const [cx, cy] = P(x, y); c.save(); c.globalAlpha = alpha; c.translate(cx, cy); c.rotate(-th);
    c.fillStyle = css('--dim'); [[0.18, 0.17], [0.18, -0.17], [-0.18, 0.17], [-0.18, -0.17]].forEach(([wx, wy]) => c.fillRect((wx - 0.06) * s, (-wy - 0.025) * s, 0.12 * s, 0.05 * s));
    c.strokeStyle = col; c.lineWidth = 2; c.strokeRect(-0.25 * s, -0.15 * s, 0.5 * s, 0.3 * s);
    c.restore(); c.globalAlpha = 1;
    arrow(c, cx, cy, cx + Math.cos(th) * 0.38 * s, cy - Math.sin(th) * 0.38 * s, col, 2, 7);
  }
  function path(c, P, pts, col, dash, wdt = 2) {
    c.strokeStyle = col; c.lineWidth = wdt; c.setLineDash(dash || []); c.beginPath();
    pts.forEach((p, i) => { const q = P(p[0], p[1]); i ? c.lineTo(...q) : c.moveTo(...q); }); c.stroke(); c.setLineDash([]);
  }
  function goals(c, P, s) {
    ROUTE.forEach(([gx, gy], i) => { const q = P(gx, gy); c.strokeStyle = css('--yellow'); c.lineWidth = 2; c.beginPath(); c.arc(q[0], q[1], Math.max(4, 0.05 * s), 0, 7); c.stroke(); text(c, 'W' + (i + 1), q[0] + 8, q[1] - 10, css('--yellow'), 11); });
  }

  /* ---------- Section 1: eyes closed vs eyes open (recorded MuJoCo runs) ---------- */
  (() => {
    const cv = $('l14cvIdea'); let g = fit(cv, 0.62);
    const V = { x0: -0.5, x1: 2.1, y0: -0.4, y1: 1.4 };
    let run = null, k = 0, token = 0, label = '';
    function draw() {
      const { ctx: c, w, h } = g, { P, s } = floor(c, w, h, V);
      goals(c, P, s);
      if (run) {
        const pts = run.path.slice(0, k + 1); path(c, P, pts, run.col);
        const p = pts[pts.length - 1]; robot(c, P, s, p[0], p[1], p[2], run.col);
        text(c, label, 10, 14, run.col, 12);
        text(c, 't = ' + (k * 0.2).toFixed(1) + ' s', 10, 32, css('--dim'), 11);
        if (k === run.path.length - 1) text(c, 'final distance to W2: ' + run.err.toFixed(3) + ' m', 10, 50, run.col, 12);
      } else { robot(c, P, s, 0, 0, 0, css('--teal')); text(c, 'press a button: the robot must reach W1, then W2', 10, 14, css('--dim'), 11); }
    }
    function go(caseKey, kind) {
      const r = REC.runs[caseKey][kind], my = ++token;
      run = { path: r.path, err: r.err, col: kind === 'open' ? css('--orange') : css('--teal') };
      label = (kind === 'open' ? 'eyes closed (open loop), ' : 'eyes open (closed loop), ') + CASE_SHORT[caseKey];
      k = 0;
      if (RM) { k = r.path.length - 1; draw(); return; }
      const t0 = performance.now();
      (function frame(now) { if (my !== token) return; k = Math.min(r.path.length - 1, Math.max(0, Math.floor((now - t0) / 50))); draw(); if (k < r.path.length - 1) requestAnimationFrame(frame); })(t0);
    }
    $('l14IdeaOpenN').addEventListener('click', () => go('nominal', 'open'));
    $('l14IdeaOpenS').addEventListener('click', () => go('slippery', 'open'));
    $('l14IdeaClosedN').addEventListener('click', () => go('nominal', 'closed'));
    $('l14IdeaClosedS').addEventListener('click', () => go('slippery', 'closed'));
    addEventListener('resize', () => { g = fit(cv, 0.62); draw(); }); addEventListener('themechange', draw);
    draw();
  })();

  /* ---------- Section 2: yaw from the quaternion and from xmat ---------- */
  (() => {
    const cv = $('l14cvPose'); let g = fit(cv, 0.8);
    let yaw = 0;
    const set = (id, v) => { const el = $(id); if (el.textContent !== v) { el.textContent = v; flash(el.closest('.ro')); } };
    function draw() {
      const { ctx: c, w, h } = g, { P, s } = floor(c, w, h, { x0: -0.6, x1: 0.6, y0: -0.5, y1: 0.55 }, 0.25);
      robot(c, P, s, 0, 0, yaw, css('--teal'));
      // yaw arc from world +x to body +x
      const o = P(0, 0), rr = 0.2 * s; c.strokeStyle = css('--yellow'); c.lineWidth = 2; c.beginPath();
      c.arc(o[0], o[1], rr, Math.min(0, -yaw), Math.max(0, -yaw)); c.stroke();
      text(c, 'yaw ' + deg(yaw).toFixed(1) + ' deg', o[0] + rr * 1.1 * Math.cos(-yaw / 2), o[1] + rr * 1.1 * Math.sin(-yaw / 2) - 10, css('--yellow'), 11);
      text(c, 'teal arrow = body +x (xmat column 0)', 10, 14, css('--teal'), 11);
      const qw = Math.cos(yaw / 2), qz = Math.sin(yaw / 2);
      const a = 2 * (qw * qz + 0 * 0), b = 1 - 2 * (0 * 0 + qz * qz);
      set('l14Qw', qw.toFixed(4)); set('l14Qz', qz.toFixed(4));
      set('l14Qa', a.toFixed(6)); set('l14Qb', b.toFixed(6));
      set('l14YawQ', Math.atan2(a, b).toFixed(6));
      set('l14Col', '[' + Math.cos(yaw).toFixed(6) + ', ' + Math.sin(yaw).toFixed(6) + ', 0]');
      set('l14YawM', Math.atan2(Math.sin(yaw), Math.cos(yaw)).toFixed(6));
      $('l14oYaw').textContent = deg(yaw).toFixed(1) + ' deg';
    }
    $('l14Yaw').addEventListener('input', () => { yaw = +$('l14Yaw').value * Math.PI / 180; draw(); });
    $('l14YawScript').addEventListener('click', () => { yaw = 1.329145; $('l14Yaw').value = deg(yaw).toFixed(0); draw(); });
    addEventListener('resize', () => { g = fit(cv, 0.8); draw(); }); addEventListener('themechange', draw);
    draw();
  })();

  /* ---------- Section 3: skid-steer, formula with geometric B vs calibrated B_eff ---------- */
  (() => {
    const cv = $('l14cvSkid'); let g = fit(cv, 0.75);
    // MuJoCo, steady averages 2-6 s (14_mobile_bases.py, part 4): v (m/s), w (rad/s)
    const MEAS = { '5,5': [0.2938, 0.0000], '-5,5': [-0.0000, 0.4460], '-2,2': [-0.0001, 0.1541], '0,5': [0.1459, 0.1954], '2,5': [0.2056, 0.1013], '4,6': [0.2934, 0.0554] };
    function trace(L, Rr, B, kv) {
      const pts = [[0, 0, 0]]; let x = 0, y = 0, th = 0; const v = kv * 0.06 * (L + Rr) / 2, w = 0.06 * (Rr - L) / B;
      for (let i = 0; i < 400; i++) { th += w * 0.01; x += v * Math.cos(th) * 0.01; y += v * Math.sin(th) * 0.01; if (i % 4 === 3) pts.push([x, y, th]); }
      return { pts, v, w };
    }
    function draw() {
      const L = +$('l14L').value, Rr = +$('l14R').value;
      $('l14oL').textContent = L.toFixed(1) + ' rad/s'; $('l14oR').textContent = Rr.toFixed(1) + ' rad/s';
      const { ctx: c, w, h } = g, { P, s } = floor(c, w, h, { x0: -0.5, x1: 1.5, y0: -0.6, y1: 0.9 });
      const geo = trace(L, Rr, 0.34, 1), cal = trace(L, Rr, K14.B, K14.KV);
      path(c, P, geo.pts, css('--orange'), [6, 5]); path(c, P, cal.pts, css('--teal'));
      const eg = geo.pts[geo.pts.length - 1], ec = cal.pts[cal.pts.length - 1];
      robot(c, P, s, eg[0], eg[1], eg[2], css('--orange'), 0.55);
      robot(c, P, s, ec[0], ec[1], ec[2], css('--teal'));
      robot(c, P, s, 0, 0, 0, css('--dim'), 0.35);
      text(c, 'after 4 s, orange dashed: B = 0.34 m', 10, 14, css('--orange'), 11);
      text(c, 'teal: B_eff = ' + K14.B.toFixed(4) + ' m', 10, 30, css('--teal'), 11);
      $('l14Vg').textContent = geo.v.toFixed(4); $('l14Wg').textContent = geo.w.toFixed(4);
      $('l14Vc').textContent = cal.v.toFixed(4); $('l14Wc').textContent = cal.w.toFixed(4);
      const m = MEAS[L + ',' + Rr];
      $('l14Vm').textContent = m ? m[0].toFixed(4) : 'not measured'; $('l14Wm').textContent = m ? m[1].toFixed(4) : 'not measured';
    }
    ['l14L', 'l14R'].forEach(id => $(id).addEventListener('input', draw));
    document.querySelectorAll('#lesson14 [data-skid]').forEach(b => b.addEventListener('click', () => { const [l, r] = b.dataset.skid.split(','); $('l14L').value = l; $('l14R').value = r; draw(); }));
    addEventListener('resize', () => { g = fit(cv, 0.75); draw(); }); addEventListener('themechange', draw);
    draw();
  })();

  /* ---------- Section 4: rangefinder ray to the wall ---------- */
  (() => {
    const cv = $('l14cvRange'); let g = fit(cv, 0.62);
    function draw() {
      const x = +$('l14Rx').value, th = +$('l14Rth').value * Math.PI / 180;
      $('l14oRx').textContent = x.toFixed(2) + ' m'; $('l14oRth').textContent = (+$('l14Rth').value).toFixed(0) + ' deg';
      const { ctx: c, w, h } = g, { P, s } = floor(c, w, h, { x0: -0.6, x1: 3.2, y0: -1.2, y1: 1.2 });
      const a = P(2.95, 1.2), b = P(3.05, -1.2); c.fillStyle = css('--dim'); c.globalAlpha = 0.5; c.fillRect(a[0], a[1], b[0] - a[0], b[1] - a[1]); c.globalAlpha = 1;
      text(c, 'wall face x = 2.95 m', a[0] - 6, b[1] - 10, css('--dim'), 10, 'right');
      const sx = x + 0.26 * Math.cos(th), sy = 0.26 * Math.sin(th), co = Math.cos(th);
      let d = -1; if (co > 1e-9) { const dd = (2.95 - sx) / co, hy = sy + dd * Math.sin(th); if (dd >= 0 && Math.abs(hy) <= 3) d = dd; }
      robot(c, P, s, x, 0, th, css('--teal'));
      const p0 = P(sx, sy), L = d > 0 ? d : 6, p1 = P(sx + L * Math.cos(th), sy + L * Math.sin(th));
      c.strokeStyle = d > 0 ? css('--purple') : css('--red'); c.lineWidth = 1.5; c.setLineDash(d > 0 ? [] : [4, 4]); c.beginPath(); c.moveTo(...p0); c.lineTo(...p1); c.stroke(); c.setLineDash([]);
      if (d > 0) { c.fillStyle = css('--purple'); c.beginPath(); c.arc(...p1, 4, 0, 7); c.fill(); }
      text(c, d > 0 ? 'rangefinder = ' + d.toFixed(4) + ' m' : 'rangefinder = -1 (no hit)', 10, 14, d > 0 ? css('--purple') : css('--red'), 12);
      $('l14Rd').textContent = d > 0 ? d.toFixed(4) : '-1';
    }
    ['l14Rx', 'l14Rth'].forEach(id => $(id).addEventListener('input', draw));
    addEventListener('resize', () => { g = fit(cv, 0.62); draw(); }); addEventListener('themechange', draw);
    draw();
  })();

  /* ---------- Section 5: the four recorded experiments ---------- */
  (() => {
    const cv = $('l14cvRuns'); let g = fit(cv, 0.62);
    let cur = 'nominal';
    function draw() {
      const { ctx: c, w, h } = g, { P, s } = floor(c, w, h, { x0: -0.5, x1: 2.6, y0: -0.4, y1: 1.4 });
      goals(c, P, s);
      const r = REC.runs[cur];
      path(c, P, r.open.path, css('--orange'), [6, 4]); path(c, P, r.closed.path, css('--teal'));
      const eo = r.open.path[r.open.path.length - 1], ec = r.closed.path[r.closed.path.length - 1];
      robot(c, P, s, eo[0], eo[1], eo[2], css('--orange'), 0.8); robot(c, P, s, ec[0], ec[1], ec[2], css('--teal'));
      text(c, 'MuJoCo, ' + CASE_SHORT[cur], 10, 14, css('--bright'), 12);
      text(c, 'dashed orange = open loop, teal = closed loop', 10, 31, css('--dim'), 11);
      $('l14RunOE').textContent = r.open.err.toFixed(4); $('l14RunOT').textContent = r.open.t.toFixed(2);
      $('l14RunCE').textContent = r.closed.err.toFixed(4); $('l14RunCT').textContent = r.closed.t.toFixed(2);
      document.querySelectorAll('#lesson14 [data-run]').forEach(b => b.classList.toggle('primary', b.dataset.run === cur));
    }
    document.querySelectorAll('#lesson14 [data-run]').forEach(b => b.addEventListener('click', () => { cur = b.dataset.run; draw(); }));
    addEventListener('resize', () => { g = fit(cv, 0.62); draw(); }); addEventListener('themechange', draw);
    draw();
  })();
})();
})();
