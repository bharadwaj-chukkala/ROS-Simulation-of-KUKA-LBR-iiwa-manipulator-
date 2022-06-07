# ROS-Simulation-of-KUKA-LBR-iiwa-manipulator

## ENPM662 Final Project (Cyberknife: A non invasive radiosurgical application of robotics)

|Team Members
|--
|Bharadwaj Chukkala
|Joseph Pranadeer Reddy Katakam
|Bhargav Kumar Soothram


## Contents
1. Part Files and Assembly
2. Package

## Dependencies
- python 3.9 (works for any python 3 version)
- Python running IDE. (I used PyCharm IDE to program the Code and Execute the Code)
- Libraries: numpy, matplotlib.pyplot, heapq, time

## How to run the code
--> Create a catkin_ws and build it, then source it

--> Download the package

--> Paste the package in the source directory

--> Build and source the workspace

--> Commands to run (open separate terminals and run in the same order)
```bash
  roslaunch Assembly_Toby template_launch.launch
```
```bash
  rosrun Assembly_Toby publisher.py
```
```bash
  rosrun Assembly_Toby subscriber.py
```

### Check out the final video here 👇
https://drive.google.com/file/d/1k4bVFm9eszw6IDoCDINYM6II19aMtiYY/view?usp=sharing

### Contact Author

Name : __Bharadwaj Chukkala__ <br>
Email : bchukkal@terpmail.umd.edu <br>

[![forthebadge](data:image/svg+xml;base64,PHN2ZyB4bWxucz0iaHR0cDovL3d3dy53My5vcmcvMjAwMC9zdmciIHdpZHRoPSIzNzIuNzEiIGhlaWdodD0iMzUiIHZpZXdCb3g9IjAgMCAzNzIuNzEgMzUiPjxyZWN0IGNsYXNzPSJzdmdfX3JlY3QiIHg9IjAiIHk9IjAiIHdpZHRoPSIxNzEuNzEiIGhlaWdodD0iMzUiIGZpbGw9IiMzMUM0RjMiLz48cmVjdCBjbGFzcz0ic3ZnX19yZWN0IiB4PSIxNjkuNzEiIHk9IjAiIHdpZHRoPSIyMDIuOTk5OTk5OTk5OTk5OTciIGhlaWdodD0iMzUiIGZpbGw9IiMzODlBRDUiLz48cGF0aCBjbGFzcz0ic3ZnX190ZXh0IiBkPSJNMTMuOTUgMTguMTlMMTMuOTUgMTguMTlMMTMuOTUgMTcuMzlRMTMuOTUgMTYuMTkgMTQuMzggMTUuMjdRMTQuODAgMTQuMzUgMTUuNjAgMTMuODVRMTYuNDAgMTMuMzUgMTcuNDUgMTMuMzVMMTcuNDUgMTMuMzVRMTguODYgMTMuMzUgMTkuNzMgMTQuMTJRMjAuNTkgMTQuODkgMjAuNzMgMTYuMjlMMjAuNzMgMTYuMjlMMTkuMjUgMTYuMjlRMTkuMTQgMTUuMzcgMTguNzEgMTQuOTZRMTguMjggMTQuNTUgMTcuNDUgMTQuNTVMMTcuNDUgMTQuNTVRMTYuNDggMTQuNTUgMTUuOTcgMTUuMjZRMTUuNDUgMTUuOTYgMTUuNDQgMTcuMzNMMTUuNDQgMTcuMzNMMTUuNDQgMTguMDlRMTUuNDQgMTkuNDcgMTUuOTMgMjAuMjBRMTYuNDMgMjAuOTIgMTcuMzggMjAuOTJMMTcuMzggMjAuOTJRMTguMjUgMjAuOTIgMTguNjkgMjAuNTNRMTkuMTMgMjAuMTQgMTkuMjUgMTkuMjJMMTkuMjUgMTkuMjJMMjAuNzMgMTkuMjJRMjAuNjAgMjAuNTkgMTkuNzIgMjEuMzVRMTguODQgMjIuMTIgMTcuMzggMjIuMTJMMTcuMzggMjIuMTJRMTYuMzYgMjIuMTIgMTUuNTkgMjEuNjNRMTQuODEgMjEuMTUgMTQuMzkgMjAuMjZRMTMuOTcgMTkuMzcgMTMuOTUgMTguMTlaTTI0Ljc3IDE4LjAwTDI0Ljc3IDE4LjAwTDI0Ljc3IDE3LjUyUTI0Ljc3IDE2LjI4IDI1LjIxIDE1LjMyUTI1LjY1IDE0LjM3IDI2LjQ2IDEzLjg2UTI3LjI3IDEzLjM1IDI4LjMxIDEzLjM1UTI5LjM1IDEzLjM1IDMwLjE2IDEzLjg1UTMwLjk2IDE0LjM1IDMxLjQwIDE1LjI5UTMxLjg0IDE2LjIzIDMxLjg1IDE3LjQ4TDMxLjg1IDE3LjQ4TDMxLjg1IDE3Ljk2UTMxLjg1IDE5LjIxIDMxLjQxIDIwLjE2UTMwLjk4IDIxLjEwIDMwLjE4IDIxLjYxUTI5LjM3IDIyLjEyIDI4LjMyIDIyLjEyTDI4LjMyIDIyLjEyUTI3LjI4IDIyLjEyIDI2LjQ3IDIxLjYxUTI1LjY2IDIxLjEwIDI1LjIyIDIwLjE3UTI0Ljc4IDE5LjIzIDI0Ljc3IDE4LjAwWk0yNi4yNSAxNy40NkwyNi4yNSAxNy45NlEyNi4yNSAxOS4zNiAyNi44MCAyMC4xM1EyNy4zNSAyMC45MCAyOC4zMiAyMC45MEwyOC4zMiAyMC45MFEyOS4zMSAyMC45MCAyOS44NCAyMC4xNVEzMC4zNyAxOS40MCAzMC4zNyAxNy45NkwzMC4zNyAxNy45NkwzMC4zNyAxNy41MVEzMC4zNyAxNi4wOSAyOS44MyAxNS4zNFEyOS4yOSAxNC41OCAyOC4zMSAxNC41OEwyOC4zMSAxNC41OFEyNy4zNSAxNC41OCAyNi44MSAxNS4zM1EyNi4yNiAxNi4wOSAyNi4yNSAxNy40NkwyNi4yNSAxNy40NlpNMzguNzcgMjJMMzYuMzEgMjJMMzYuMzEgMTMuNDdMMzguODMgMTMuNDdRMzkuOTYgMTMuNDcgNDAuODQgMTMuOTdRNDEuNzIgMTQuNDggNDIuMjAgMTUuNDBRNDIuNjggMTYuMzMgNDIuNjggMTcuNTJMNDIuNjggMTcuNTJMNDIuNjggMTcuOTVRNDIuNjggMTkuMTYgNDIuMTkgMjAuMDhRNDEuNzEgMjEuMDAgNDAuODIgMjEuNTBRMzkuOTIgMjIgMzguNzcgMjJMMzguNzcgMjJaTTM3LjgwIDE0LjY2TDM3LjgwIDIwLjgyTDM4Ljc2IDIwLjgyUTM5LjkzIDIwLjgyIDQwLjU1IDIwLjA5UTQxLjE4IDE5LjM2IDQxLjE5IDE3Ljk5TDQxLjE5IDE3Ljk5TDQxLjE5IDE3LjUyUTQxLjE5IDE2LjEzIDQwLjU4IDE1LjQwUTM5Ljk4IDE0LjY2IDM4LjgzIDE0LjY2TDM4LjgzIDE0LjY2TDM3LjgwIDE0LjY2Wk01Mi43MiAyMkw0Ny4xNCAyMkw0Ny4xNCAxMy40N0w1Mi42OCAxMy40N0w1Mi42OCAxNC42Nkw0OC42MiAxNC42Nkw0OC42MiAxNy4wMkw1Mi4xMyAxNy4wMkw1Mi4xMyAxOC4xOUw0OC42MiAxOC4xOUw0OC42MiAyMC44Mkw1Mi43MiAyMC44Mkw1Mi43MiAyMlpNNTkuMzcgMjJMNTYuOTEgMjJMNTYuOTEgMTMuNDdMNTkuNDMgMTMuNDdRNjAuNTYgMTMuNDcgNjEuNDQgMTMuOTdRNjIuMzIgMTQuNDggNjIuODAgMTUuNDBRNjMuMjggMTYuMzMgNjMuMjggMTcuNTJMNjMuMjggMTcuNTJMNjMuMjggMTcuOTVRNjMuMjggMTkuMTYgNjIuNzkgMjAuMDhRNjIuMzEgMjEuMDAgNjEuNDIgMjEuNTBRNjAuNTIgMjIgNTkuMzcgMjJMNTkuMzcgMjJaTTU4LjQwIDE0LjY2TDU4LjQwIDIwLjgyTDU5LjM2IDIwLjgyUTYwLjUzIDIwLjgyIDYxLjE1IDIwLjA5UTYxLjc4IDE5LjM2IDYxLjc5IDE3Ljk5TDYxLjc5IDE3Ljk5TDYxLjc5IDE3LjUyUTYxLjc5IDE2LjEzIDYxLjE5IDE1LjQwUTYwLjU4IDE0LjY2IDU5LjQzIDE0LjY2TDU5LjQzIDE0LjY2TDU4LjQwIDE0LjY2Wk03NS4yOCAyMkw3My44MSAyMkw3My44MSAxMy40N0w3NS4yOCAxMy40N0w3NS4yOCAyMlpNODEuNTggMjJMODAuMDkgMjJMODAuMDkgMTMuNDdMODEuNTggMTMuNDdMODUuMzkgMTkuNTRMODUuMzkgMTMuNDdMODYuODYgMTMuNDdMODYuODYgMjJMODUuMzggMjJMODEuNTggMTUuOTVMODEuNTggMjJaTTk5LjA2IDIyTDk3LjU4IDIyTDk3LjU4IDEzLjQ3TDEwMC44NCAxMy40N1ExMDIuMjcgMTMuNDcgMTAzLjExIDE0LjIxUTEwMy45NSAxNC45NiAxMDMuOTUgMTYuMThMMTAzLjk1IDE2LjE4UTEwMy45NSAxNy40NCAxMDMuMTMgMTguMTNRMTAyLjMxIDE4LjgzIDEwMC44MiAxOC44M0wxMDAuODIgMTguODNMOTkuMDYgMTguODNMOTkuMDYgMjJaTTk5LjA2IDE0LjY2TDk5LjA2IDE3LjY0TDEwMC44NCAxNy42NFExMDEuNjMgMTcuNjQgMTAyLjA1IDE3LjI3UTEwMi40NyAxNi45MCAxMDIuNDcgMTYuMTlMMTAyLjQ3IDE2LjE5UTEwMi40NyAxNS41MCAxMDIuMDQgMTUuMDlRMTAxLjYyIDE0LjY4IDEwMC44OCAxNC42NkwxMDAuODggMTQuNjZMOTkuMDYgMTQuNjZaTTExMC4yOCAxOC44NkwxMDcuNDEgMTMuNDdMMTA5LjA2IDEzLjQ3TDExMS4wMiAxNy41MUwxMTIuOTggMTMuNDdMMTE0LjYyIDEzLjQ3TDExMS43NiAxOC44NkwxMTEuNzYgMjJMMTEwLjI4IDIyTDExMC4yOCAxOC44NlpNMTIwLjIyIDE0LjY2TDExNy41OCAxNC42NkwxMTcuNTggMTMuNDdMMTI0LjM1IDEzLjQ3TDEyNC4zNSAxNC42NkwxMjEuNjkgMTQuNjZMMTIxLjY5IDIyTDEyMC4yMiAyMkwxMjAuMjIgMTQuNjZaTTEyOS41OSAyMkwxMjguMTEgMjJMMTI4LjExIDEzLjQ3TDEyOS41OSAxMy40N0wxMjkuNTkgMTcuMDJMMTMzLjQwIDE3LjAyTDEzMy40MCAxMy40N0wxMzQuODggMTMuNDdMMTM0Ljg4IDIyTDEzMy40MCAyMkwxMzMuNDAgMTguMjFMMTI5LjU5IDE4LjIxTDEyOS41OSAyMlpNMTM5LjM1IDE4LjAwTDEzOS4zNSAxOC4wMEwxMzkuMzUgMTcuNTJRMTM5LjM1IDE2LjI4IDEzOS43OSAxNS4zMlExNDAuMjMgMTQuMzcgMTQxLjA0IDEzLjg2UTE0MS44NSAxMy4zNSAxNDIuODkgMTMuMzVRMTQzLjkzIDEzLjM1IDE0NC43NCAxMy44NVExNDUuNTQgMTQuMzUgMTQ1Ljk4IDE1LjI5UTE0Ni40MiAxNi4yMyAxNDYuNDMgMTcuNDhMMTQ2LjQzIDE3LjQ4TDE0Ni40MyAxNy45NlExNDYuNDMgMTkuMjEgMTQ1Ljk5IDIwLjE2UTE0NS41NiAyMS4xMCAxNDQuNzYgMjEuNjFRMTQzLjk1IDIyLjEyIDE0Mi45MCAyMi4xMkwxNDIuOTAgMjIuMTJRMTQxLjg2IDIyLjEyIDE0MS4wNSAyMS42MVExNDAuMjQgMjEuMTAgMTM5LjgwIDIwLjE3UTEzOS4zNiAxOS4yMyAxMzkuMzUgMTguMDBaTTE0MC44MyAxNy40NkwxNDAuODMgMTcuOTZRMTQwLjgzIDE5LjM2IDE0MS4zOCAyMC4xM1ExNDEuOTMgMjAuOTAgMTQyLjkwIDIwLjkwTDE0Mi45MCAyMC45MFExNDMuODkgMjAuOTAgMTQ0LjQyIDIwLjE1UTE0NC45NSAxOS40MCAxNDQuOTUgMTcuOTZMMTQ0Ljk1IDE3Ljk2TDE0NC45NSAxNy41MVExNDQuOTUgMTYuMDkgMTQ0LjQxIDE1LjM0UTE0My44NyAxNC41OCAxNDIuODkgMTQuNThMMTQyLjg5IDE0LjU4UTE0MS45MyAxNC41OCAxNDEuMzkgMTUuMzNRMTQwLjg0IDE2LjA5IDE0MC44MyAxNy40NkwxNDAuODMgMTcuNDZaTTE1Mi4zOCAyMkwxNTAuODkgMjJMMTUwLjg5IDEzLjQ3TDE1Mi4zOCAxMy40N0wxNTYuMTkgMTkuNTRMMTU2LjE5IDEzLjQ3TDE1Ny42NiAxMy40N0wxNTcuNjYgMjJMMTU2LjE4IDIyTDE1Mi4zOCAxNS45NUwxNTIuMzggMjJaIiBmaWxsPSIjRkZGRkZGIi8+PHBhdGggY2xhc3M9InN2Z19fdGV4dCIgZD0iTTE4OC40NCAyMkwxODMuOTAgMjJMMTgzLjkwIDEzLjYwTDE4OC4yMCAxMy42MFExODkuODAgMTMuNjAgMTkwLjY0IDE0LjE5UTE5MS40OSAxNC43OSAxOTEuNDkgMTUuNzlMMTkxLjQ5IDE1Ljc5UTE5MS40OSAxNi4zOSAxOTEuMTkgMTYuODdRMTkwLjg5IDE3LjM0IDE5MC4zNSAxNy42MkwxOTAuMzUgMTcuNjJRMTkxLjA4IDE3Ljg3IDE5MS40OCAxOC40MVExOTEuODkgMTguOTQgMTkxLjg5IDE5LjcwTDE5MS44OSAxOS43MFExOTEuODkgMjAuODAgMTkxLjAwIDIxLjQwUTE5MC4xMSAyMiAxODguNDQgMjJMMTg4LjQ0IDIyWk0xODYuMjUgMTguNThMMTg2LjI1IDIwLjI4TDE4OC4yNSAyMC4yOFExODkuNDkgMjAuMjggMTg5LjQ5IDE5LjQzTDE4OS40OSAxOS40M1ExODkuNDkgMTguNTggMTg4LjI1IDE4LjU4TDE4OC4yNSAxOC41OEwxODYuMjUgMTguNThaTTE4Ni4yNSAxNS4zMUwxODYuMjUgMTYuOTRMMTg3Ljg4IDE2Ljk0UTE4OS4wOCAxNi45NCAxODkuMDggMTYuMTJMMTg5LjA4IDE2LjEyUTE4OS4wOCAxNS4zMSAxODcuODggMTUuMzFMMTg3Ljg4IDE1LjMxTDE4Ni4yNSAxNS4zMVpNMTk4LjczIDE4Ljk1TDE5NS41MyAxMy42MEwxOTguMDQgMTMuNjBMMjAwLjAzIDE2Ljk0TDIwMi4wMiAxMy42MEwyMDQuMzIgMTMuNjBMMjAxLjExIDE4Ljk5TDIwMS4xMSAyMkwxOTguNzMgMjJMMTk4LjczIDE4Ljk1Wk0yMTkuOTIgMjJMMjE1LjM4IDIyTDIxNS4zOCAxMy42MEwyMTkuNjggMTMuNjBRMjIxLjI5IDEzLjYwIDIyMi4xMyAxNC4xOVEyMjIuOTcgMTQuNzkgMjIyLjk3IDE1Ljc5TDIyMi45NyAxNS43OVEyMjIuOTcgMTYuMzkgMjIyLjY3IDE2Ljg3UTIyMi4zOCAxNy4zNCAyMjEuODQgMTcuNjJMMjIxLjg0IDE3LjYyUTIyMi41NiAxNy44NyAyMjIuOTcgMTguNDFRMjIzLjM3IDE4Ljk0IDIyMy4zNyAxOS43MEwyMjMuMzcgMTkuNzBRMjIzLjM3IDIwLjgwIDIyMi40OCAyMS40MFEyMjEuNjAgMjIgMjE5LjkyIDIyTDIxOS45MiAyMlpNMjE3Ljc0IDE4LjU4TDIxNy43NCAyMC4yOEwyMTkuNzMgMjAuMjhRMjIwLjk4IDIwLjI4IDIyMC45OCAxOS40M0wyMjAuOTggMTkuNDNRMjIwLjk4IDE4LjU4IDIxOS43MyAxOC41OEwyMTkuNzMgMTguNThMMjE3Ljc0IDE4LjU4Wk0yMTcuNzQgMTUuMzFMMjE3Ljc0IDE2Ljk0TDIxOS4zNiAxNi45NFEyMjAuNTcgMTYuOTQgMjIwLjU3IDE2LjEyTDIyMC41NyAxNi4xMlEyMjAuNTcgMTUuMzEgMjE5LjM2IDE1LjMxTDIxOS4zNiAxNS4zMUwyMTcuNzQgMTUuMzFaTTIzMC40NyAyMkwyMjguMDkgMjJMMjI4LjA5IDEzLjYwTDIzMC40NyAxMy42MEwyMzAuNDcgMTYuNzZMMjMzLjcxIDE2Ljc2TDIzMy43MSAxMy42MEwyMzYuMDggMTMuNjBMMjM2LjA4IDIyTDIzMy43MSAyMkwyMzMuNzEgMTguNzJMMjMwLjQ3IDE4LjcyTDIzMC40NyAyMlpNMjQyLjY2IDIyTDI0MC4yNCAyMkwyNDMuOTQgMTMuNjBMMjQ2LjI5IDEzLjYwTDI1MC4wMCAyMkwyNDcuNTQgMjJMMjQ2Ljg3IDIwLjM3TDI0My4zMiAyMC4zN0wyNDIuNjYgMjJaTTI0NS4xMCAxNS45M0wyNDQuMDIgMTguNjFMMjQ2LjE4IDE4LjYxTDI0NS4xMCAxNS45M1pNMjU2LjU0IDIyTDI1NC4xNiAyMkwyNTQuMTYgMTMuNjBMMjU4LjAwIDEzLjYwUTI1OS4xNCAxMy42MCAyNTkuOTggMTMuOThRMjYwLjgyIDE0LjM1IDI2MS4yOCAxNS4wNlEyNjEuNzMgMTUuNzYgMjYxLjczIDE2LjcxTDI2MS43MyAxNi43MVEyNjEuNzMgMTcuNjIgMjYxLjMxIDE4LjMwUTI2MC44OCAxOC45OCAyNjAuMDkgMTkuMzZMMjYwLjA5IDE5LjM2TDI2MS45MCAyMkwyNTkuMzUgMjJMMjU3LjgzIDE5Ljc3TDI1Ni41NCAxOS43N0wyNTYuNTQgMjJaTTI1Ni41NCAxNS40N0wyNTYuNTQgMTcuOTNMMjU3Ljg1IDE3LjkzUTI1OC41OSAxNy45MyAyNTguOTYgMTcuNjFRMjU5LjMzIDE3LjI5IDI1OS4zMyAxNi43MUwyNTkuMzMgMTYuNzFRMjU5LjMzIDE2LjEyIDI1OC45NiAxNS43OVEyNTguNTkgMTUuNDcgMjU3Ljg1IDE1LjQ3TDI1Ny44NSAxNS40N0wyNTYuNTQgMTUuNDdaTTI2Ny45NCAyMkwyNjUuNTEgMjJMMjY5LjIyIDEzLjYwTDI3MS41NyAxMy42MEwyNzUuMjggMjJMMjcyLjgxIDIyTDI3Mi4xNSAyMC4zN0wyNjguNjAgMjAuMzdMMjY3Ljk0IDIyWk0yNzAuMzggMTUuOTNMMjY5LjI5IDE4LjYxTDI3MS40NSAxOC42MUwyNzAuMzggMTUuOTNaTTI4My40MSAyMkwyNzkuNDQgMjJMMjc5LjQ0IDEzLjYwTDI4My40MSAxMy42MFEyODQuNzkgMTMuNjAgMjg1Ljg2IDE0LjEyUTI4Ni45MiAxNC42MyAyODcuNTEgMTUuNThRMjg4LjEwIDE2LjUzIDI4OC4xMCAxNy44MEwyODguMTAgMTcuODBRMjg4LjEwIDE5LjA3IDI4Ny41MSAyMC4wMlEyODYuOTIgMjAuOTcgMjg1Ljg2IDIxLjQ4UTI4NC43OSAyMiAyODMuNDEgMjJMMjgzLjQxIDIyWk0yODEuODEgMTUuNTBMMjgxLjgxIDIwLjEwTDI4My4zMSAyMC4xMFEyODQuMzkgMjAuMTAgMjg1LjA1IDE5LjQ5UTI4NS43MCAxOC44OCAyODUuNzAgMTcuODBMMjg1LjcwIDE3LjgwUTI4NS43MCAxNi43MiAyODUuMDUgMTYuMTFRMjg0LjM5IDE1LjUwIDI4My4zMSAxNS41MEwyODMuMzEgMTUuNTBMMjgxLjgxIDE1LjUwWk0yOTQuOTEgMjJMMjkyLjE5IDEzLjYwTDI5NC42MyAxMy42MEwyOTYuMzIgMTguOTZMMjk4LjEwIDEzLjYwTDMwMC4yOCAxMy42MEwzMDEuOTggMTkuMDFMMzAzLjc0IDEzLjYwTDMwNi4wMSAxMy42MEwzMDMuMjggMjJMMzAwLjc0IDIyTDI5OS4xMyAxNi44OUwyOTcuNDUgMjJMMjk0LjkxIDIyWk0zMTEuOTQgMjJMMzA5LjUxIDIyTDMxMy4yMiAxMy42MEwzMTUuNTYgMTMuNjBMMzE5LjI4IDIyTDMxNi44MSAyMkwzMTYuMTUgMjAuMzdMMzEyLjYwIDIwLjM3TDMxMS45NCAyMlpNMzE0LjM3IDE1LjkzTDMxMy4yOSAxOC42MUwzMTUuNDUgMTguNjFMMzE0LjM3IDE1LjkzWk0zMjIuMzUgMjAuOTNMMzIyLjM1IDIwLjkzTDMyMy42NSAxOS40MFEzMjQuMzIgMjAuMjcgMzI1LjA5IDIwLjI3TDMyNS4wOSAyMC4yN1EzMjUuMTAgMjAuMjcgMzI1LjEwIDIwLjI3TDMyNS4xMCAyMC4yN1EzMjUuNjIgMjAuMjcgMzI1Ljg5IDE5Ljk2UTMyNi4xNiAxOS42NSAzMjYuMTYgMTkuMDVMMzI2LjE2IDE5LjA1TDMyNi4xNiAxNS40NEwzMjMuMjYgMTUuNDRMMzIzLjI2IDEzLjYwTDMyOC41MSAxMy42MEwzMjguNTEgMTguOTFRMzI4LjUxIDIwLjU0IDMyNy42OSAyMS4zNlEzMjYuODcgMjIuMTcgMzI1LjI3IDIyLjE3TDMyNS4yNyAyMi4xN1EzMjQuMzUgMjIuMTcgMzIzLjU5IDIxLjg1UTMyMi44NCAyMS41MyAzMjIuMzUgMjAuOTNaTTM0MC4xNCAxNy44MEwzNDAuMTQgMTcuODBRMzQwLjE0IDE2LjU0IDM0MC43NCAxNS41NFEzNDEuMzQgMTQuNTUgMzQyLjM5IDEzLjk5UTM0My40NCAxMy40MyAzNDQuNzYgMTMuNDNMMzQ0Ljc2IDEzLjQzUTM0NS45MiAxMy40MyAzNDYuODQgMTMuODRRMzQ3Ljc2IDE0LjI1IDM0OC4zOCAxNS4wMkwzNDguMzggMTUuMDJMMzQ2Ljg2IDE2LjM5UTM0Ni4wNSAxNS40MCAzNDQuODggMTUuNDBMMzQ0Ljg4IDE1LjQwUTM0NC4yMCAxNS40MCAzNDMuNjcgMTUuNzBRMzQzLjEzIDE2IDM0Mi44NCAxNi41NFEzNDIuNTQgMTcuMDkgMzQyLjU0IDE3LjgwTDM0Mi41NCAxNy44MFEzNDIuNTQgMTguNTEgMzQyLjg0IDE5LjA1UTM0My4xMyAxOS42MCAzNDMuNjcgMTkuOTBRMzQ0LjIwIDIwLjIwIDM0NC44OCAyMC4yMEwzNDQuODggMjAuMjBRMzQ2LjA1IDIwLjIwIDM0Ni44NiAxOS4yMkwzNDYuODYgMTkuMjJMMzQ4LjM4IDIwLjU4UTM0Ny43NyAyMS4zNSAzNDYuODQgMjEuNzZRMzQ1LjkyIDIyLjE3IDM0NC43NiAyMi4xN0wzNDQuNzYgMjIuMTdRMzQzLjQ0IDIyLjE3IDM0Mi4zOSAyMS42MVEzNDEuMzQgMjEuMDUgMzQwLjc0IDIwLjA1UTM0MC4xNCAxOS4wNiAzNDAuMTQgMTcuODBaTTM1NS4yOSAyMkwzNTIuOTEgMjJMMzUyLjkxIDEzLjYwTDM1NS4yOSAxMy42MEwzNTUuMjkgMTYuNzZMMzU4LjUzIDE2Ljc2TDM1OC41MyAxMy42MEwzNjAuOTAgMTMuNjBMMzYwLjkwIDIyTDM1OC41MyAyMkwzNTguNTMgMTguNzJMMzU1LjI5IDE4LjcyTDM1NS4yOSAyMloiIGZpbGw9IiNGRkZGRkYiIHg9IjE4Mi43MSIvPjwvc3ZnPg==)](https://forthebadge.com)





