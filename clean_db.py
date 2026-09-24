import sqlite3
import os

# Path to your database (relative to where you run the script)
DB_PATH = './data/fleet_ledger.db'

# REPLACE 'tasks' with your actual table name
TABLE_NAME = 'tasks_ledger' 

def clean_database():
    if not os.path.exists(DB_PATH):
        print(f"Error: Database not found at {DB_PATH}")
        return

    try:
        # Connect to the SQLite database
        conn = sqlite3.connect(DB_PATH)
        cursor = conn.cursor()

        # Delete rows where Task_completion_time is NULL or an empty string
        query = f"DELETE FROM {TABLE_NAME} WHERE Task_completion_time IS NULL OR trim(Task_completion_time) = '';"
        cursor.execute(query)

        # Commit the transaction
        conn.commit()

        # Output the result
        deleted_rows = cursor.rowcount
        print(f"Successfully deleted {deleted_rows} incomplete tasks from '{TABLE_NAME}'.")

    except sqlite3.Error as e:
        print(f"Database error: {e}")
        
    finally:
        if 'conn' in locals() and conn:
            conn.close()

if __name__ == "__main__":
    clean_database()