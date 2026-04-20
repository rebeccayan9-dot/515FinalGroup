#!/usr/bin/env python3
"""Train a Random Forest behavior classifier from labeled SafeStep CSVs."""

import pandas as pd
import numpy as np
from pathlib import Path
from sklearn.ensemble import RandomForestClassifier
from sklearn.model_selection import train_test_split
from sklearn.metrics import classification_report, confusion_matrix
from sklearn.preprocessing import LabelEncoder
import joblib

DATA_DIR = Path(__file__).parent / "data"
MODEL_PATH = Path(__file__).parent / "model.pkl"
WINDOW = 10       # rows per sample (~2 seconds at 5 Hz)
STEP = 5          # sliding window step
FEATURES = ["lidar_cm", "front_cm", "left_cm", "right_cm"]
STATS = ["mean", "std", "min", "max"]


def load_data():
    dfs = []
    for csv in DATA_DIR.glob("*.csv"):
        df = pd.read_csv(csv)
        if "label" not in df.columns:
            print(f"  skipping {csv.name} (no label column)")
            continue
        df = df.dropna(subset=FEATURES)
        # Replace "OUT" lidar readings with NaN then forward-fill
        df["lidar_cm"] = pd.to_numeric(df["lidar_cm"], errors="coerce")
        df["lidar_cm"] = df["lidar_cm"].ffill().fillna(400)
        dfs.append(df)
        print(f"  loaded {csv.name}: {len(df)} rows, label={df['label'].iloc[0]}")
    if not dfs:
        raise FileNotFoundError(f"No labeled CSVs found in {DATA_DIR}")
    return pd.concat(dfs, ignore_index=True)


def make_windows(df):
    X, y = [], []
    for start in range(0, len(df) - WINDOW + 1, STEP):
        window = df.iloc[start : start + WINDOW]
        if window["label"].nunique() > 1:
            continue  # skip windows that span two behaviors
        feats = []
        for col in FEATURES:
            vals = window[col].values.astype(float)
            feats += [vals.mean(), vals.std(), vals.min(), vals.max()]
        X.append(feats)
        y.append(window["label"].iloc[0])
    cols = [f"{c}_{s}" for c in FEATURES for s in STATS]
    return pd.DataFrame(X, columns=cols), np.array(y)


def main():
    print("Loading CSVs...")
    df = load_data()
    print(f"\nTotal rows: {len(df)}, labels: {df['label'].value_counts().to_dict()}\n")

    print("Building windows...")
    X, y = make_windows(df)
    print(f"Samples: {len(X)}, features: {X.shape[1]}\n")

    le = LabelEncoder()
    y_enc = le.fit_transform(y)

    X_train, X_test, y_train, y_test = train_test_split(
        X, y_enc, test_size=0.2, random_state=42, stratify=y_enc
    )

    print("Training Random Forest...")
    clf = RandomForestClassifier(n_estimators=100, random_state=42, n_jobs=-1)
    clf.fit(X_train, y_train)

    y_pred = clf.predict(X_test)
    print("\n--- Results ---")
    print(classification_report(y_test, y_pred, target_names=le.classes_))
    print("Confusion matrix (rows=actual, cols=predicted):")
    print(pd.DataFrame(
        confusion_matrix(y_test, y_pred),
        index=le.classes_, columns=le.classes_
    ))

    print("\nTop features by importance:")
    importances = pd.Series(clf.feature_importances_, index=X.columns)
    print(importances.sort_values(ascending=False).head(8).to_string())

    joblib.dump({"model": clf, "label_encoder": le}, MODEL_PATH)
    print(f"\nModel saved to {MODEL_PATH}")


if __name__ == "__main__":
    main()
