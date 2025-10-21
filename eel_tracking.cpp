#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>

using namespace cv;
using namespace std;

// Constants
const int BG_SUBTRACTOR_HISTORY = 500;
const double BG_SUBTRACTOR_THRESHOLD = 16.0;
const bool DETECT_SHADOWS = false;
const double MIN_AREA = 1500.0;
const double MAX_AREA = 5000.0;
const double MAX_ASPECT_RATIO = 0.33;
const double MIN_MOTION_DISTANCE = 2.0; // Minimum distance to consider motion
const int MIN_FRAMES_EEL = 5; // frames before confirming a track as an eel
const int MAX_FRAMES_MISSED = 50; // frames before deleting a track

struct Track {
	int id;
    Point2f position;
    int framesTracked;
    int lastFrameSeen;
	bool eel_detected = false; // Indicates if the track is considered as an eel
};

int main(int argc, char** argv) {
    if (argc < 2) {
        cout << "Usage: " << argv[0] << " <video-file>" << endl;
        return -1;
    }

    // Read video
    VideoCapture cap(argv[1]);
    if (!cap.isOpened()) {
        cerr << "Erreur : impossible d'ouvrir la vidéo." << endl;
        return -1;
    }

    // Create video file
    int frame_width = (int)cap.get(CAP_PROP_FRAME_WIDTH);
    int frame_height = (int)cap.get(CAP_PROP_FRAME_HEIGHT);
    double fps = cap.get(CAP_PROP_FPS);
    string output_filename = "Vid_tracking.mp4";
    int fourcc = VideoWriter::fourcc('m', 'p', 'v', '4');
    VideoWriter outputVideo(output_filename, fourcc, fps, Size(frame_width, frame_height));
    if (!outputVideo.isOpened()) {
        cerr << "Erreur : impossible de créer le fichier vidéo." << endl;
        return -1;
    }

    // Create CSV file
    ofstream csv_file("Vid_tracking.csv");
    if (!csv_file.is_open()) {
        cerr << "Erreur : impossible de créer le fichier CSV." << endl;
        return -1;
    }
    csv_file << "frame,timestamp_sec,track_id,x,y" << endl;
    csv_file.flush();

	Ptr<BackgroundSubtractor> bgSub = createBackgroundSubtractorMOG2(BG_SUBTRACTOR_HISTORY, BG_SUBTRACTOR_THRESHOLD, DETECT_SHADOWS); // Background subtractor
	vector<Track> tracks; // List of active tracks

	// Variable initialization
    int nextTrackID = 1;
    int eelNumber = 0;
    int currentFrame = 0;
    Mat frame, gray, fgMask;

    while (true) {
        cap >> frame;
        if (frame.empty()) break;
        currentFrame++;

        cvtColor(frame, gray, COLOR_BGR2GRAY);
        GaussianBlur(gray, gray, Size(7, 7), 1.5);
		bgSub->apply(gray, fgMask); // Apply background subtraction

		// Morphological operations
        morphologyEx(fgMask, fgMask, MORPH_OPEN, Mat::ones(5, 5, CV_8U));
        morphologyEx(fgMask, fgMask, MORPH_CLOSE, Mat::ones(11, 11, CV_8U));

        vector<vector<Point>> contours;
        findContours(fgMask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

		// Extract centers of potential eels from the current frame
        vector<Point2f> currentCenters;
        vector<RotatedRect> currentBoxes;
        for (auto& cnt : contours) {
            double area = contourArea(cnt);
            if (area < MIN_AREA|| area > MAX_AREA) continue;
            RotatedRect box = minAreaRect(cnt);
            float ratio = (float)min(box.size.width, box.size.height) / max(box.size.width, box.size.height);
            if (ratio > MAX_ASPECT_RATIO) continue;

            drawContours(frame, vector<std::vector<cv::Point>>{cnt}, -1, Scalar(255, 0, 0), 2); // Contour of the potential eel

            currentCenters.push_back(box.center);
            currentBoxes.push_back(box);
        }

        // Associate each detection with an existing track (nearest neighbor)
        vector<bool> assigned(currentCenters.size(), false); // Indicates if a center has been assigned to a track
        for (auto& track : tracks) {
            float minDist = 1e9;
            int bestIdx = -1;
			for (int i = 0; i < currentCenters.size(); ++i) { // Iterate over the centers of the current frame
                if (assigned[i]) continue;
				float d = norm(track.position - currentCenters[i]); // Euclidean distance between the track and the current center
                if (d < minDist) {
                    minDist = d;
					bestIdx = i; // Index of the closest center
                }
            }

			if (bestIdx != -1 && minDist < cap.get(CAP_PROP_FRAME_WIDTH)/4) {
                if (minDist < MIN_MOTION_DISTANCE) {
                    assigned[bestIdx] = true; // Immobile center (likely a fixed object)
					continue;
                }

                track.position = currentCenters[bestIdx];
                track.lastFrameSeen = currentFrame;
                track.framesTracked++;
                assigned[bestIdx] = true;

				// Display track
				circle(frame, track.position, 3, Scalar(0, 0, 255), -1); // Track's center
                putText(frame, "id :" + to_string(track.id), track.position + Point2f(10, 0), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1);

                if (track.framesTracked > MIN_FRAMES_EEL || track.eel_detected ==  true) {
                    if (!track.eel_detected) {
                        track.eel_detected = true; // Mark the track as an eel
						eelNumber++;
					}
					rectangle(frame, currentBoxes[bestIdx].boundingRect(), Scalar(0, 255, 0), 2); // Bounding box around the detected eel

                    string texte = "Anguille";
                    int fontFace = FONT_HERSHEY_SIMPLEX;
                    double fontScale = 0.5;
                    int thickness = 1;
                    int baseline = 0;

                    // Text size
                    Size textSize = getTextSize(texte, fontFace, fontScale, thickness, &baseline);

                    // Center the text below the rectangle
                    int x = currentBoxes[bestIdx].boundingRect().x + (currentBoxes[bestIdx].boundingRect().width - textSize.width) / 2;
                    int y = currentBoxes[bestIdx].boundingRect().y + currentBoxes[bestIdx].boundingRect().height + textSize.height + 5; // 5 pixels below the rectangle

                    Point textOrigin(x, y);

                    // Display the centered text below the rectangle
                    putText(frame, texte, textOrigin, fontFace, fontScale, Scalar(0, 255, 0), thickness);

                    // Write CSV file
                    double timestamp = cap.get(CAP_PROP_POS_MSEC) / 1000.0;
                    csv_file << currentFrame << "," << timestamp << "," << track.id << "," << track.position.x << "," << track.position.y << endl;
                    csv_file.flush();
				}
            }
        }

        // Create new tracks for unassigned detections
        for (int i = 0; i < currentCenters.size(); ++i) {
            if (!assigned[i]) {
                Track newTrack;
                newTrack.id = nextTrackID++;
                newTrack.position = currentCenters[i];
                newTrack.lastFrameSeen = currentFrame;
                newTrack.framesTracked = 1;
                tracks.push_back(newTrack);

                // Display new track
                circle(frame, newTrack.position, 3, Scalar(0, 0, 255), -1);
                putText(frame, "id :" + to_string(newTrack.id), newTrack.position + Point2f(10, 0), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1);
            }
        }

        // Delete old tracks (not seen for MAX_FRAMES_MISSED frames)
        tracks.erase(remove_if(tracks.begin(), tracks.end(), [currentFrame](const Track& t) {return currentFrame - t.lastFrameSeen > MAX_FRAMES_MISSED;}), tracks.end());

        // Display frame and write video file
        imshow("Tracking anguilles", frame);
        outputVideo.write(frame);
        if (waitKey(1) == 27) break; // ESC to exit

        // Display the number of processed frames in the console
        int numberOfFrames = cap.get(CAP_PROP_FRAME_COUNT);
        int currentFrame = (int)cap.get(CAP_PROP_POS_FRAMES);
        cout << "Frame " << currentFrame << "/" << numberOfFrames << endl;
    }

    csv_file.close();
    cap.release();
    cv::destroyAllWindows();
    return 0;
}