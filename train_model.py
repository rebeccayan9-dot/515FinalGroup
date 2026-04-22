#!/usr/bin/env python3
"""Train RF behavior classifier and export C++ code for ESP32S3 inference."""

import pandas as pd
import numpy as np
from pathlib import Path
from sklearn.ensemble import RandomForestClassifier
from sklearn.model_selection import train_test_split, cross_val_score
from sklearn.metrics import classification_report, confusion_matrix
from sklearn.preprocessing import LabelEncoder
import joblib

DATA_DIR   = Path(__file__).parent / "data"
MODEL_PATH = Path(__file__).parent / "model.pkl"
CPP_PATH   = Path(__file__).parent / "src" / "classifier.h"

WINDOW = 5   # samples per window (~1.5s at 3Hz)
STEP   = 2   # sliding step

FEATURES = [
    "lidar_cm",
    "front_cm", "left_cm", "right_cm",
    "ax_g", "ay_g", "az_g",
    "gx_dps", "gy_dps", "gz_dps",
]
STATS = ["mean", "std", "min", "max"]


def load_and_clean(csv_path):
    df = pd.read_csv(csv_path)
    if "label" not in df.columns:
        return None

    # lidar: "OUT" → NaN → forward fill → fallback 400
    df["lidar_cm"] = pd.to_numeric(df["lidar_cm"], errors="coerce")
    df["lidar_cm"] = df["lidar_cm"].ffill().fillna(400)

    # ultrasonic -1.0 → NaN → forward fill → fallback 400
    for col in ["front_cm", "left_cm", "right_cm"]:
        df[col] = df[col].replace(-1.0, np.nan).ffill().fillna(400)

    # drop rows missing IMU
    df = df.dropna(subset=FEATURES)
    return df if len(df) >= WINDOW else None


def make_windows(df):
    """Slide a window over one continuous recording."""
    X, y = [], []
    for start in range(0, len(df) - WINDOW + 1, STEP):
        w = df.iloc[start : start + WINDOW]
        if w["label"].nunique() > 1:
            continue
        feats = []
        for col in FEATURES:
            v = w[col].values.astype(float)
            feats += [v.mean(), v.std(ddof=0), v.min(), v.max()]
        X.append(feats)
        y.append(w["label"].iloc[0])
    return X, y


def main():
    print("Loading data...")
    all_X, all_y = [], []
    for csv in sorted(DATA_DIR.glob("*.csv")):
        df = load_and_clean(csv)
        if df is None:
            continue
        X, y = make_windows(df)
        all_X.extend(X)
        all_y.extend(y)
        print(f"  {csv.name}: {len(df)} rows → {len(X)} windows  [{y[0] if y else '?'}]")

    feature_names = [f"{c}_{s}" for c in FEATURES for s in STATS]
    X = pd.DataFrame(all_X, columns=feature_names)
    y = np.array(all_y)

    print(f"\nTotal windows: {len(X)}")
    for label, n in sorted(pd.Series(y).value_counts().items()):
        print(f"  {label:<20} {n}")

    le = LabelEncoder()
    y_enc = le.fit_transform(y)
    print(f"\nClasses: {list(le.classes_)}")

    # Train
    X_train, X_test, y_train, y_test = train_test_split(
        X, y_enc, test_size=0.2, random_state=42, stratify=y_enc
    )
    clf = RandomForestClassifier(n_estimators=100, max_depth=10,
                                 random_state=42, n_jobs=-1)
    clf.fit(X_train, y_train)

    # Evaluate
    y_pred = clf.predict(X_test)
    print("\n--- Test set results ---")
    print(classification_report(y_test, y_pred, target_names=le.classes_))
    print("Confusion matrix:")
    print(pd.DataFrame(
        confusion_matrix(y_test, y_pred),
        index=le.classes_, columns=le.classes_
    ))

    cv = cross_val_score(clf, X, y_enc, cv=5, scoring="accuracy")
    print(f"\n5-fold CV accuracy: {cv.mean():.3f} ± {cv.std():.3f}")

    print("\nTop 10 features:")
    imp = pd.Series(clf.feature_importances_, index=feature_names)
    print(imp.sort_values(ascending=False).head(10).to_string())

    # Save model
    joblib.dump({"model": clf, "label_encoder": le,
                 "features": feature_names, "window": WINDOW, "step": STEP},
                MODEL_PATH)
    print(f"\nModel saved → {MODEL_PATH}")

    # Export C++ for ESP32S3
    try:
        from micromlgen import port
        classmap = {int(le.transform([c])[0]): c for c in le.classes_}
        cpp = port(clf, classmap=classmap)
        CPP_PATH.write_text(cpp)
        print(f"C++ classifier exported → {CPP_PATH}")
    except ImportError:
        print("\nmicromlgen not installed. Run: pip install micromlgen")
        print("Then re-run this script to export C++ code.")


if __name__ == "__main__":
    main()
